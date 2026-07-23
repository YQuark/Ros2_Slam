"""ROS-independent, layout-aware Host command quality supervisor."""

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Optional, Tuple

from robot_chassis_model.wheel_layout import WheelLayout


class SupervisorLevel(IntEnum):
    NORMAL = 0
    WARN = 1
    DEGRADED = 2
    CRITICAL = 3


@dataclass(frozen=True)
class SupervisorConfig:
    enabled: bool = True
    wheel_pair_warn_mps: float = 0.10
    wheel_pair_critical_mps: float = 0.25
    tracking_warn_mps: float = 0.10
    tracking_critical_mps: float = 0.30
    tracking_min_target_mps: float = 0.08
    yaw_warn_radps: float = 0.25
    yaw_critical_radps: float = 1.0
    zero_command_linear_mps: float = 0.01
    zero_command_angular_radps: float = 0.02
    unexpected_linear_mps: float = 0.05
    unexpected_angular_radps: float = 0.25
    warn_score: float = 0.40
    degraded_score: float = 0.60
    critical_score: float = 0.85
    critical_hold_sec: float = 0.50
    clear_score: float = 0.30
    clear_hold_sec: float = 1.0
    degraded_min_scale: float = 0.25


@dataclass(frozen=True)
class SupervisorResult:
    score: float
    level: SupervisorLevel
    command_scale: float
    release_host_candidate: bool
    reason: str


def observation_requires_release(
    *,
    schema_version: int,
    expected_schema_version: int,
    encoder_anomaly_mask: int,
    enabled_mask: int = 0x0F,
    speed_valid_mask: int = 0x0F,
) -> bool:
    return (
        int(schema_version) != int(expected_schema_version)
        or not WheelLayout(enabled_mask, speed_valid_mask, encoder_anomaly_mask).complete
    )


def _normalize(value: float, low: float, high: float) -> float:
    if high <= low:
        return 1.0 if abs(value) > low else 0.0
    return max(0.0, min(1.0, (abs(float(value)) - low) / (high - low)))


class MotionSupervisor:
    def __init__(self, config: SupervisorConfig = SupervisorConfig()) -> None:
        self.config = config
        self.level = SupervisorLevel.NORMAL
        self._critical_since: Optional[float] = None
        self._clear_since: Optional[float] = None

    def update(
        self,
        *,
        now_sec: float,
        command_vx: float,
        command_wz: float,
        wheel_speeds: Tuple[float, float, float, float],
        wheel_targets: Tuple[float, float, float, float],
        enabled_mask: int = 0x0F,
        speed_valid_mask: int = 0x0F,
        anomaly_mask: int = 0,
        feedback_vx: float,
        wheel_wz: float,
        gyro_z: Optional[float],
    ) -> SupervisorResult:
        c = self.config
        if not c.enabled:
            self.level = SupervisorLevel.NORMAL
            self._critical_since = self._clear_since = None
            return SupervisorResult(0.0, self.level, 1.0, False, "disabled")
        layout = WheelLayout(enabled_mask, speed_valid_mask, anomaly_mask)
        pair = layout.pair_disagreement(wheel_speeds)
        tracking = 0.0
        for index in layout.left_indices + layout.right_indices:
            target, actual = wheel_targets[index], wheel_speeds[index]
            if abs(target) >= c.tracking_min_target_mps:
                tracking = max(tracking, abs(target - actual))
        components: Dict[str, float] = {
            "wheel_pair": _normalize(pair, c.wheel_pair_warn_mps, c.wheel_pair_critical_mps),
            "tracking": _normalize(tracking, c.tracking_warn_mps, c.tracking_critical_mps),
            "yaw": (
                0.0
                if gyro_z is None
                else _normalize(wheel_wz - gyro_z, c.yaw_warn_radps, c.yaw_critical_radps)
            ),
            "unexpected_motion": (
                1.0
                if abs(command_vx) < c.zero_command_linear_mps
                and abs(command_wz) < c.zero_command_angular_radps
                and (
                    abs(feedback_vx) > c.unexpected_linear_mps
                    or abs(wheel_wz) > c.unexpected_angular_radps
                    or (gyro_z is not None and abs(gyro_z) > c.unexpected_angular_radps)
                )
                else 0.0
            ),
        }
        reason = max(components, key=components.get)
        score = components[reason]
        if score >= c.critical_score:
            self._critical_since = now_sec if self._critical_since is None else self._critical_since
            self._clear_since = None
            if now_sec - self._critical_since >= c.critical_hold_sec:
                self.level = SupervisorLevel.CRITICAL
        elif score >= c.degraded_score:
            self._critical_since = self._clear_since = None
            self.level = SupervisorLevel.DEGRADED
        elif score >= c.warn_score:
            self._critical_since = self._clear_since = None
            self.level = SupervisorLevel.WARN
        elif score < c.clear_score:
            self._critical_since = None
            self._clear_since = now_sec if self._clear_since is None else self._clear_since
            if now_sec - self._clear_since >= c.clear_hold_sec:
                self.level = SupervisorLevel.NORMAL
        scale = 1.0
        if self.level >= SupervisorLevel.DEGRADED:
            span = max(c.critical_score - c.degraded_score, 1e-6)
            fraction = max(0.0, min(1.0, (score - c.degraded_score) / span))
            scale = 0.60 + fraction * (c.degraded_min_scale - 0.60)
        return SupervisorResult(
            score, self.level, scale, self.level is SupervisorLevel.CRITICAL, reason
        )
