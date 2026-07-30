"""ROS-independent Upper-v3 firmware semantics for PTY integration tests."""

from __future__ import annotations

import struct
from collections import deque
from typing import Deque, Optional


ACK_SESSION_VALID = 1 << 0
ACK_RECEIVED = 1 << 1
ACK_APPLIED = 1 << 2
ACK_DUPLICATE = 1 << 3
ACK_REJECTED = 1 << 4


def _forward(new: int, old: int) -> bool:
    delta = (int(new) - int(old)) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


class FirmwareV3Emulator:
    """Strict command/session/watchdog model; transport framing stays outside."""

    def __init__(self, *, host_watchdog_sec: float = 0.2, enabled_mask: int = 0x06):
        self.host_watchdog_sec = float(host_watchdog_sec)
        self.enabled_mask = int(enabled_mask) & 0x0F
        self.retired_sessions: Deque[int] = deque(maxlen=8)
        self.status_sequence = 0
        self.sample_time_ms = 0
        self.session = 0
        self.received_sequence = 0
        self.applied_sequence = 0
        self.ack_flags = 0
        self.reject_reason = 0
        self.last_payload: Optional[bytes] = None
        self.last_host_at: Optional[float] = None
        self.host_enabled = False
        self.estop = False
        self.fault = False
        self.line_enabled = False

    def apply_velocity(self, payload: bytes, now_sec: float) -> bool:
        if len(payload) != 23:
            return self._reject(1)
        version, vx, wz, enable, _mode, session, sequence = struct.unpack("<BffBBQI", payload)
        if version != 3 or session == 0 or session in self.retired_sessions:
            return self._reject(2)
        if self.session == session:
            if sequence == self.received_sequence:
                if payload != self.last_payload:
                    return self._reject(3)
                self.last_host_at = float(now_sec)
                self.ack_flags = ACK_SESSION_VALID | ACK_RECEIVED | ACK_APPLIED | ACK_DUPLICATE
                self.reject_reason = 0
                return True
            if not _forward(sequence, self.received_sequence):
                return self._reject(4)
        elif enable:
            return self._reject(5)
        else:
            if self.session:
                self.retired_sessions.append(self.session)
            self.session = session
        self.received_sequence = sequence
        self.last_payload = bytes(payload)
        self.last_host_at = float(now_sec)
        if enable and (self.estop or self.fault):
            return self._reject(6)
        self.host_enabled = bool(enable)
        self.applied_sequence = sequence
        self.ack_flags = ACK_SESSION_VALID | ACK_RECEIVED | ACK_APPLIED
        self.reject_reason = 0
        return True

    def tick(self, now_sec: float) -> None:
        if (
            self.host_enabled
            and self.last_host_at is not None
            and float(now_sec) - self.last_host_at > self.host_watchdog_sec
        ):
            self.host_enabled = False

    def set_estop(self) -> None:
        self.estop = True
        self.host_enabled = False

    def set_fault(self) -> None:
        self.fault = True
        self.host_enabled = False

    def clear_fault(self) -> bool:
        if self.estop:
            return False
        self.fault = False
        return True

    def reboot(self) -> None:
        if self.session:
            self.retired_sessions.append(self.session)
        self.session = 0
        self.received_sequence = self.applied_sequence = 0
        self.ack_flags = self.reject_reason = 0
        self.last_payload = self.last_host_at = None
        self.host_enabled = False

    def status_payload(self, dt_sec: float = 0.02) -> bytes:
        self.status_sequence = (self.status_sequence + 1) & 0xFFFFFFFF
        self.sample_time_ms = (self.sample_time_ms + int(round(dt_sec * 1000))) & 0xFFFFFFFF
        payload = bytearray(92)
        flags = (
            (1 if self.estop else 0) | (2 if self.fault else 0) | (4 if self.line_enabled else 0)
        )
        payload[:4] = bytes((3, flags, 1 if self.host_enabled else 0, self.enabled_mask))
        struct.pack_into("<H", payload, 12, 12000)
        payload[62:65] = bytes((self.enabled_mask, 0, 0))
        struct.pack_into(
            "<IIQII",
            payload,
            65,
            self.status_sequence,
            self.sample_time_ms,
            self.session,
            self.received_sequence,
            self.applied_sequence,
        )
        payload[89] = self.reject_reason
        payload[91] = self.ack_flags
        return bytes(payload)

    @staticmethod
    def hello_payload() -> bytes:
        return (
            bytes((3, 1))
            + struct.pack("<I", 0x1F)
            + bytes.fromhex("bc472cc874e930aaed6eb8e7de73b41a2563dd85")
            + struct.pack("<II", 0x00020000, 0)
        )

    def _reject(self, reason: int) -> bool:
        self.reject_reason = int(reason) & 0xFF
        self.ack_flags = ACK_SESSION_VALID | ACK_RECEIVED | ACK_REJECTED
        return False
