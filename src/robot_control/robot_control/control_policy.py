from dataclasses import dataclass
import math
from typing import Dict, Iterable, Mapping, Optional, Tuple


DEFAULT_PRIORITIES: Mapping[str, int] = {
    "teleop": 100,
    "test": 80,
    "nav": 60,
}


class InvalidCommandError(ValueError):
    pass


def require_finite(value: float, name: str) -> float:
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise InvalidCommandError(f"{name} is not numeric") from exc
    if not math.isfinite(converted):
        raise InvalidCommandError(f"{name} is not finite")
    return converted


@dataclass(frozen=True)
class Command:
    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class SelectedCommand:
    source: str
    command: Command
    active: bool


@dataclass(frozen=True)
class SourceConfig:
    research_sources: Tuple[str, ...] = ()

    def __init__(self, research_sources: Iterable[str] = ()) -> None:
        object.__setattr__(
            self,
            "research_sources",
            tuple(name.strip() for name in research_sources if name.strip()),
        )

    def topic_map(self) -> Dict[str, str]:
        topics = {
            "teleop": "/cmd_vel/teleop",
            "nav": "/cmd_vel/nav",
            "test": "/cmd_vel/test",
        }
        for name in self.research_sources:
            topics[f"research/{name}"] = f"/cmd_vel/research/{name}"
        return topics

    def priorities(self) -> Dict[str, int]:
        priorities = dict(DEFAULT_PRIORITIES)
        for name in self.research_sources:
            priorities[f"research/{name}"] = 40
        return priorities


class CommandMux:
    def __init__(
        self,
        *,
        source_config: SourceConfig,
        linear_limit: float,
        angular_limit: float,
        timeout_sec: float,
        input_linear_abs_max: float = 5.0,
        input_angular_abs_max: float = 20.0,
    ) -> None:
        self._priorities = source_config.priorities()
        self._linear_limit = abs(require_finite(linear_limit, "linear_limit"))
        self._angular_limit = abs(require_finite(angular_limit, "angular_limit"))
        self._timeout_sec = max(require_finite(timeout_sec, "timeout_sec"), 0.0)
        self._input_linear_abs_max = abs(
            require_finite(input_linear_abs_max, "input_linear_abs_max")
        )
        self._input_angular_abs_max = abs(
            require_finite(input_angular_abs_max, "input_angular_abs_max")
        )
        self._commands: Dict[str, tuple[Command, float]] = {}
        self.reject_count = 0

    def update(self, source: str, command: Command, now_sec: float) -> bool:
        if source not in self._priorities:
            return False
        try:
            linear_x = require_finite(command.linear_x, "linear_x")
            angular_z = require_finite(command.angular_z, "angular_z")
            stamp = require_finite(now_sec, "command timestamp")
            if abs(linear_x) > self._input_linear_abs_max:
                raise InvalidCommandError(
                    f"linear_x exceeds absolute input limit {self._input_linear_abs_max}"
                )
            if abs(angular_z) > self._input_angular_abs_max:
                raise InvalidCommandError(
                    f"angular_z exceeds absolute input limit {self._input_angular_abs_max}"
                )
        except InvalidCommandError:
            self.reject(source)
            raise

        self._commands[source] = (
            Command(
                linear_x=_clamp(linear_x, -self._linear_limit, self._linear_limit),
                angular_z=_clamp(angular_z, -self._angular_limit, self._angular_limit),
            ),
            stamp,
        )
        return True

    def reject(self, source: str) -> None:
        self._commands.pop(source, None)
        self.reject_count += 1

    def select(self, now_sec: float) -> SelectedCommand:
        now_sec = require_finite(now_sec, "selection timestamp")
        active = []
        for source, (command, stamp) in self._commands.items():
            age = now_sec - stamp
            if 0.0 <= age <= self._timeout_sec:
                active.append((self._priorities[source], stamp, source, command))
        if not active:
            return SelectedCommand(source="idle", command=Command(0.0, 0.0), active=False)

        _, _, source, command = max(active, key=lambda item: (item[0], item[1]))
        return SelectedCommand(source=source, command=command, active=True)


class MotionLimiter:
    def __init__(
        self,
        *,
        max_linear_accel: float,
        max_angular_accel: float,
        max_linear_jerk: float,
        max_angular_jerk: float,
        max_dt_sec: float = 0.1,
    ) -> None:
        self.max_linear_accel = abs(require_finite(max_linear_accel, "max_linear_accel"))
        self.max_angular_accel = abs(require_finite(max_angular_accel, "max_angular_accel"))
        self.max_linear_jerk = abs(require_finite(max_linear_jerk, "max_linear_jerk"))
        self.max_angular_jerk = abs(require_finite(max_angular_jerk, "max_angular_jerk"))
        self.max_dt_sec = abs(require_finite(max_dt_sec, "max_dt_sec"))
        self.reset()

    def reset(self) -> None:
        self.current = Command(0.0, 0.0)
        self._linear_accel = 0.0
        self._angular_accel = 0.0
        self._last_time_sec: Optional[float] = None

    def limit(self, target: Command, *, now_sec: float) -> Command:
        now_sec = require_finite(now_sec, "motion limiter timestamp")
        target = Command(
            require_finite(target.linear_x, "target linear_x"),
            require_finite(target.angular_z, "target angular_z"),
        )
        if self._last_time_sec is None:
            self._last_time_sec = now_sec
            return self.current
        dt = now_sec - self._last_time_sec
        if dt <= 0.0:
            return self.current
        dt = min(dt, self.max_dt_sec)
        self._last_time_sec = now_sec

        linear, self._linear_accel = self._step_axis(
            self.current.linear_x,
            target.linear_x,
            self._linear_accel,
            self.max_linear_accel,
            self.max_linear_jerk,
            dt,
        )
        angular, self._angular_accel = self._step_axis(
            self.current.angular_z,
            target.angular_z,
            self._angular_accel,
            self.max_angular_accel,
            self.max_angular_jerk,
            dt,
        )
        self.current = Command(linear, angular)
        return self.current

    @staticmethod
    def _step_axis(current, target, acceleration, max_accel, max_jerk, dt):
        desired_accel = _clamp((target - current) / dt, -max_accel, max_accel)
        acceleration = _clamp(
            desired_accel,
            acceleration - max_jerk * dt,
            acceleration + max_jerk * dt,
        )
        step = acceleration * dt
        remaining = target - current
        if remaining == 0.0 or (step * remaining > 0.0 and abs(step) > abs(remaining)):
            return target, 0.0
        return current + step, acceleration


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))
