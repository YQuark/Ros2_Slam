"""Final command validation and hard safety envelope for the serial bridge."""

import math
from dataclasses import dataclass
from typing import Optional


PROTOCOL_VERSION_V2 = 2
FAULT_STATUS_MASK = 0x03


class CommandRejected(ValueError):
    pass


@dataclass(frozen=True)
class GuardedCommand:
    vx: float
    wz: float


class CommandGuard:
    def __init__(
        self,
        *,
        hard_max_linear_mps: float,
        hard_max_angular_radps: float,
        max_command_age_sec: float,
        status_timeout_sec: float,
    ) -> None:
        self.hard_max_linear_mps = self._finite_nonnegative(
            hard_max_linear_mps, "hard_max_linear_mps"
        )
        self.hard_max_angular_radps = self._finite_nonnegative(
            hard_max_angular_radps, "hard_max_angular_radps"
        )
        self.max_command_age_sec = self._finite_nonnegative(
            max_command_age_sec, "max_command_age_sec"
        )
        self.status_timeout_sec = self._finite_nonnegative(
            status_timeout_sec, "status_timeout_sec"
        )

    def validate(
        self,
        *,
        vx: float,
        wz: float,
        command_stamp_sec: float,
        now_sec: float,
        status_age_sec: Optional[float],
        drive_permitted: bool,
        status_flags: int,
        protocol_version: int,
    ) -> GuardedCommand:
        vx = self._finite(vx, "linear velocity")
        wz = self._finite(wz, "angular velocity")
        command_stamp_sec = self._finite(command_stamp_sec, "command timestamp")
        now_sec = self._finite(now_sec, "current timestamp")
        command_age = now_sec - command_stamp_sec
        if command_age < 0.0:
            raise CommandRejected("command timestamp is in the future")
        if command_age > self.max_command_age_sec:
            raise CommandRejected("command is stale")

        if status_age_sec is None:
            raise CommandRejected("STATUS unavailable")
        status_age_sec = self._finite(status_age_sec, "STATUS age")
        if status_age_sec < 0.0 or status_age_sec > self.status_timeout_sec:
            raise CommandRejected("STATUS stale")
        if int(protocol_version) != PROTOCOL_VERSION_V2:
            raise CommandRejected("protocol version is not v2")
        if int(status_flags) & FAULT_STATUS_MASK:
            raise CommandRejected("STATUS reports ESTOP or fault-stop")
        if not drive_permitted:
            raise CommandRejected("bridge state does not permit drive")

        return GuardedCommand(
            vx=max(-self.hard_max_linear_mps, min(self.hard_max_linear_mps, vx)),
            wz=max(-self.hard_max_angular_radps, min(self.hard_max_angular_radps, wz)),
        )

    @staticmethod
    def _finite(value: float, name: str) -> float:
        try:
            converted = float(value)
        except (TypeError, ValueError) as exc:
            raise CommandRejected(f"{name} is not numeric") from exc
        if not math.isfinite(converted):
            raise CommandRejected(f"{name} is not finite")
        return converted

    @classmethod
    def _finite_nonnegative(cls, value: float, name: str) -> float:
        return max(cls._finite(value, name), 0.0)
