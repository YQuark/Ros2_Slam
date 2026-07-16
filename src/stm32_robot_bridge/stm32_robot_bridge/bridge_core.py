"""ROS- and serial-independent bridge event core."""

from __future__ import annotations

import math
import secrets
from dataclasses import dataclass, replace
from typing import Optional, Tuple

from .bridge_state import BridgeState, BridgeStateMachine
from .protocol_v3 import (
    ACK_APPLIED,
    REQUIRED_CAPABILITIES,
    STATUS_FLAG_ESTOP,
    STATUS_FLAG_FAULT_STOP,
    HelloPayload,
    StatusPayload,
    sequence_is_forward,
)


@dataclass(frozen=True)
class FirmwareSnapshot:
    received_at: float
    status: StatusPayload


@dataclass(frozen=True)
class RuntimeSnapshot:
    state: BridgeState = BridgeState.DISCONNECTED
    wire_session_id: int = 0
    wire_sent_sequence: int = 0
    command_session_id: int = 0
    command_sequence: int = 0
    selected_source: int = 0
    target_vx: float = 0.0
    target_wz: float = 0.0
    firmware: Optional[FirmwareSnapshot] = None
    hello: Optional[HelloPayload] = None
    last_status_at: Optional[float] = None
    last_command_at: Optional[float] = None
    rearm_required: bool = False


@dataclass(frozen=True)
class ValidatedCommand:
    vx: float
    wz: float
    enable: bool
    source: int
    session_id: int
    sequence: int


class BridgeCore:
    def __init__(
        self,
        *,
        hard_max_linear_mps: float,
        hard_max_angular_radps: float,
        command_timeout_sec: float,
        status_timeout_sec: float,
        max_command_age_sec: float,
        required_capabilities: int = REQUIRED_CAPABILITIES,
    ) -> None:
        self.hard_max_linear_mps = abs(float(hard_max_linear_mps))
        self.hard_max_angular_radps = abs(float(hard_max_angular_radps))
        self.command_timeout_sec = float(command_timeout_sec)
        self.status_timeout_sec = float(status_timeout_sec)
        self.max_command_age_sec = float(max_command_age_sec)
        self.required_capabilities = int(required_capabilities)
        self.machine = BridgeStateMachine()
        self.snapshot = RuntimeSnapshot()

    def on_connected(self, session_id: Optional[int] = None) -> RuntimeSnapshot:
        self.machine.on_serial_opened()
        wire_session = int(session_id if session_id is not None else secrets.randbits(64)) or 1
        self.snapshot = RuntimeSnapshot(state=self.machine.state, wire_session_id=wire_session)
        return self.snapshot

    def on_disconnected(self) -> RuntimeSnapshot:
        self.machine.on_disconnected()
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            firmware=None,
            hello=None,
            last_status_at=None,
            target_vx=0.0,
            target_wz=0.0,
            last_command_at=None,
            rearm_required=True,
        )
        return self.snapshot

    def on_hello(self, hello: HelloPayload) -> bool:
        if (
            hello.version != 3
            or (hello.capabilities & self.required_capabilities) != self.required_capabilities
        ):
            return False
        self.snapshot = replace(self.snapshot, hello=hello)
        return True

    def on_startup_released(self) -> None:
        self.machine.on_settled()
        self.snapshot = replace(self.snapshot, state=self.machine.state)

    def on_status(self, status: StatusPayload, now_sec: float) -> bool:
        if status.version != 3 or self.snapshot.hello is None:
            return False
        previous = self.snapshot.firmware
        if previous is not None and not sequence_is_forward(
            status.status_sequence, previous.status.status_sequence
        ):
            return False
        self.machine.on_valid_status(status.status_flags)
        ack_current = status.last_received_session_id == self.snapshot.wire_session_id and bool(
            status.command_ack_flags & ACK_APPLIED
        )
        if self.machine.state is BridgeState.READY and not ack_current:
            self.machine.state = BridgeState.WAIT_STATUS
        if status.status_flags & (STATUS_FLAG_ESTOP | STATUS_FLAG_FAULT_STOP):
            self.snapshot = replace(
                self.snapshot, rearm_required=True, target_vx=0.0, target_wz=0.0
            )
        firmware = FirmwareSnapshot(float(now_sec), status)
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            firmware=firmware,
            last_status_at=float(now_sec),
        )
        return True

    def accept_command(
        self,
        *,
        vx: float,
        wz: float,
        enable: bool,
        source: int,
        session_id: int,
        sequence: int,
        command_stamp_sec: float,
        now_ros_sec: float,
        now_monotonic: float,
    ) -> ValidatedCommand:
        values = (vx, wz, command_stamp_sec, now_ros_sec)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("command contains non-finite value")
        age = float(now_ros_sec) - float(command_stamp_sec)
        if age < 0.0 or age > self.max_command_age_sec:
            raise ValueError("command timestamp is invalid or stale")
        session_id = int(session_id) & 0xFFFFFFFFFFFFFFFF
        sequence = int(sequence) & 0xFFFFFFFF
        if session_id == 0:
            raise ValueError("session_id must be non-zero")
        if self.snapshot.command_session_id == session_id:
            if sequence == self.snapshot.command_sequence:
                raise ValueError("duplicate ROS command sequence")
            if not sequence_is_forward(sequence, self.snapshot.command_sequence):
                raise ValueError("out-of-order ROS command sequence")
        if not enable:
            self.machine.on_disable_command()
            self.snapshot = replace(
                self.snapshot,
                state=self.machine.state,
                command_session_id=session_id,
                command_sequence=sequence,
                selected_source=0,
                target_vx=0.0,
                target_wz=0.0,
                last_command_at=None,
                rearm_required=False,
            )
            return ValidatedCommand(0.0, 0.0, False, 0, session_id, sequence)
        if self.snapshot.rearm_required:
            raise ValueError("disable-enable rearm is required")
        status_age = (
            None
            if self.snapshot.last_status_at is None
            else now_monotonic - self.snapshot.last_status_at
        )
        if status_age is None or status_age < 0.0 or status_age > self.status_timeout_sec:
            raise ValueError("STATUS is unavailable or stale")
        if self.snapshot.hello is None or not self.machine.can_drive:
            raise ValueError("bridge state does not permit drive")
        vx = max(-self.hard_max_linear_mps, min(self.hard_max_linear_mps, float(vx)))
        wz = max(-self.hard_max_angular_radps, min(self.hard_max_angular_radps, float(wz)))
        self.machine.on_drive_enabled()
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            command_session_id=session_id,
            command_sequence=sequence,
            selected_source=int(source) & 0xFF,
            target_vx=vx,
            target_wz=wz,
            last_command_at=float(now_monotonic),
        )
        return ValidatedCommand(vx, wz, True, int(source) & 0xFF, session_id, sequence)

    def tick(self, now_sec: float) -> Tuple[bool, str]:
        if (
            self.snapshot.last_status_at is not None
            and now_sec - self.snapshot.last_status_at > self.status_timeout_sec
        ):
            self.machine.on_status_timeout()
            self.snapshot = replace(
                self.snapshot,
                state=self.machine.state,
                target_vx=0.0,
                target_wz=0.0,
                last_command_at=None,
                rearm_required=True,
            )
            return True, "status_timeout"
        if (
            self.snapshot.last_command_at is not None
            and now_sec - self.snapshot.last_command_at > self.command_timeout_sec
        ):
            self.machine.on_disable_command()
            self.snapshot = replace(
                self.snapshot,
                state=self.machine.state,
                target_vx=0.0,
                target_wz=0.0,
                last_command_at=None,
                rearm_required=True,
            )
            return True, "command_timeout"
        return False, ""
