"""ROS- and serial-independent bridge event core."""

from __future__ import annotations

import math
import secrets
from collections import deque
from dataclasses import dataclass, replace
from enum import IntEnum
from typing import Optional, Tuple

from .bridge_state import BridgeState, BridgeStateMachine
from .protocol_v3 import (
    ACK_APPLIED,
    ACK_RECEIVED,
    ACK_REJECTED,
    ACK_SESSION_VALID,
    REQUIRED_CAPABILITIES,
    STATUS_FLAG_ESTOP,
    STATUS_FLAG_FAULT_STOP,
    HelloPayload,
    StatusPayload,
    sequence_is_forward,
)


REARM_TRANSPORT = 1 << 0
REARM_STATUS_TIMEOUT = 1 << 1
REARM_ACK_TIMEOUT = 1 << 2
REARM_FIRMWARE_REJECT = 1 << 3
REARM_ESTOP = 1 << 4
REARM_FAULT = 1 << 5
REARM_COMMAND_TIMEOUT = 1 << 6
REARM_PROTOCOL_INCOMPATIBLE = 1 << 9
REARM_HOST_REPLAY = 1 << 10

INCOMPAT_PROTOCOL = 1 << 0
INCOMPAT_SCHEMA = 1 << 1
INCOMPAT_CAPABILITIES = 1 << 2


class StatusDisposition(IntEnum):
    INVALID = 0
    NEW = 1
    DUPLICATE = 2
    OUT_OF_ORDER = 3


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
    rearm_reason_flags: int = 0
    protocol_compatible: bool = False
    incompatibility_flags: int = 0
    reset_generation: int = 0


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
        self._retired_host_epochs: deque[int] = deque(maxlen=16)
        self._host_disable_observed = False

    def on_connected(self, session_id: Optional[int] = None) -> RuntimeSnapshot:
        if self.snapshot.command_session_id:
            self._retired_host_epochs.append(self.snapshot.command_session_id)
        self.machine.on_serial_opened()
        wire_session = int(session_id if session_id is not None else secrets.randbits(64)) or 1
        # Every transport generation starts closed.  The wire-side release sent by
        # the adapter is necessary but is not an upper-layer enable edge; a fresh
        # HostMotionCommand(enable=false) must still be observed before motion.
        self.snapshot = RuntimeSnapshot(
            state=self.machine.state,
            wire_session_id=wire_session,
            rearm_required=True,
            rearm_reason_flags=REARM_TRANSPORT,
            reset_generation=(self.snapshot.reset_generation + 1) & 0xFFFFFFFF,
        )
        self._host_disable_observed = False
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
            rearm_reason_flags=REARM_TRANSPORT,
            protocol_compatible=False,
            incompatibility_flags=0,
        )
        return self.snapshot

    def on_hello(self, hello: HelloPayload) -> bool:
        flags = 0
        if hello.version != 3:
            flags |= INCOMPAT_PROTOCOL
        if hello.schema_version != 1:
            flags |= INCOMPAT_SCHEMA
        if (hello.capabilities & self.required_capabilities) != self.required_capabilities:
            flags |= INCOMPAT_CAPABILITIES
        compatible = flags == 0
        if compatible:
            self.machine.on_hello_compatible()
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            hello=hello,
            protocol_compatible=compatible,
            incompatibility_flags=flags,
            rearm_required=True,
            rearm_reason_flags=(
                self.snapshot.rearm_reason_flags
                if compatible
                else self.snapshot.rearm_reason_flags | REARM_PROTOCOL_INCOMPATIBLE
            ),
        )
        return compatible

    def on_status(self, status: StatusPayload, now_sec: float) -> StatusDisposition:
        if (
            status.version != 3
            or self.snapshot.hello is None
            or not self.snapshot.protocol_compatible
        ):
            return StatusDisposition.INVALID
        previous = self.snapshot.firmware
        if previous is not None:
            if status.status_sequence == previous.status.status_sequence:
                return StatusDisposition.DUPLICATE
            if not sequence_is_forward(status.status_sequence, previous.status.status_sequence):
                return StatusDisposition.OUT_OF_ORDER
        self.machine.on_valid_status(status.status_flags)
        expected = self.machine.expected_disable_sequence
        required_ack = ACK_SESSION_VALID | ACK_RECEIVED | ACK_APPLIED
        ack_current = bool(
            expected is not None
            and status.last_received_session_id == self.snapshot.wire_session_id
            and status.last_received_sequence == expected
            and status.last_applied_sequence == expected
            and status.command_ack_flags & required_ack == required_ack
            and not status.command_ack_flags & ACK_REJECTED
            and status.last_reject_reason == 0
        )
        if expected is not None and ack_current and self.machine.on_disable_applied(expected):
            self.snapshot = replace(self.snapshot, rearm_required=False, rearm_reason_flags=0)
        if status.status_flags & (STATUS_FLAG_ESTOP | STATUS_FLAG_FAULT_STOP):
            reason = 0
            if status.status_flags & STATUS_FLAG_ESTOP:
                reason |= REARM_ESTOP
            if status.status_flags & STATUS_FLAG_FAULT_STOP:
                reason |= REARM_FAULT
            self.snapshot = replace(
                self.snapshot,
                rearm_required=True,
                rearm_reason_flags=self.snapshot.rearm_reason_flags | reason,
                target_vx=0.0,
                target_wz=0.0,
            )
        firmware = FirmwareSnapshot(float(now_sec), status)
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            firmware=firmware,
            last_status_at=float(now_sec),
        )
        return StatusDisposition.NEW

    def require_rearm(self, reason_flags: int) -> None:
        self.snapshot = replace(
            self.snapshot,
            rearm_required=True,
            rearm_reason_flags=self.snapshot.rearm_reason_flags | int(reason_flags),
            target_vx=0.0,
            target_wz=0.0,
            last_command_at=None,
        )

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
        # Header/ROS time is recording metadata. Safety freshness starts at the
        # monotonic callback receipt below and is unaffected by /clock.
        values = (vx, wz, now_monotonic)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("command contains non-finite value")
        session_id = int(session_id) & 0xFFFFFFFFFFFFFFFF
        sequence = int(sequence) & 0xFFFFFFFF
        if session_id == 0:
            raise ValueError("session_id must be non-zero")
        current_epoch = self.snapshot.command_session_id
        epoch_changed = current_epoch not in (0, session_id)
        if session_id in self._retired_host_epochs:
            self.require_rearm(REARM_HOST_REPLAY)
            raise ValueError("retired ROS command epoch")
        if current_epoch == session_id:
            if sequence == self.snapshot.command_sequence:
                raise ValueError("duplicate ROS command sequence")
            if not sequence_is_forward(sequence, self.snapshot.command_sequence):
                raise ValueError("out-of-order ROS command sequence")
        elif enable and not self._host_disable_observed:
            self.require_rearm(REARM_HOST_REPLAY)
            raise ValueError("new command epoch requires a preceding disable")
        if enable and current_epoch == session_id and self._host_disable_observed:
            self.require_rearm(REARM_HOST_REPLAY)
            raise ValueError("enable after disable requires a new command epoch")
        if epoch_changed:
            self._retired_host_epochs.append(current_epoch)
        if not enable:
            self._host_disable_observed = True
            self.snapshot = replace(
                self.snapshot,
                state=self.machine.state,
                command_session_id=session_id,
                command_sequence=sequence,
                selected_source=0,
                target_vx=0.0,
                target_wz=0.0,
                last_command_at=None,
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
        self._host_disable_observed = False
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

    def on_disable_sent(self, sequence: int) -> None:
        self.machine.on_disable_sent(sequence)
        self.snapshot = replace(
            self.snapshot,
            state=self.machine.state,
            rearm_required=(
                self.snapshot.rearm_required
                or self.machine.state is BridgeState.WAIT_POST_CLEAR_DISABLE_ACK
            ),
        )

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
                rearm_reason_flags=self.snapshot.rearm_reason_flags | REARM_STATUS_TIMEOUT,
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
                rearm_reason_flags=self.snapshot.rearm_reason_flags | REARM_COMMAND_TIMEOUT,
            )
            return True, "command_timeout"
        return False, ""
