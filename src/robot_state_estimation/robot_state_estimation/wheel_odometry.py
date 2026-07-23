"""Layout-aware differential-drive odometry and covariance propagation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from robot_chassis_model.wheel_layout import WheelLayout


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class EncoderOdometryUpdate:
    pose: Pose2D
    vx: float
    wz: float
    dt: float
    trusted: bool
    integrated: bool
    left_distance: float
    right_distance: float
    pose_covariance: Tuple[float, ...]


def signed_int32_delta(current: int, previous: int) -> int:
    return ((int(current) - int(previous) + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)


class EncoderOdometry:
    def __init__(
        self,
        *,
        wheel_radius_m: float,
        track_width_m: float,
        counts_per_revolution: float,
        max_dt_sec: float = 0.25,
        linear_scale: float = 1.0,
        angular_scale: float = 1.0,
        angular_sign: float = 1.0,
        wheel_variance_floor_m2: float = 0.000025,
        wheel_variance_per_meter: float = 0.0025,
    ) -> None:
        if wheel_radius_m <= 0.0 or counts_per_revolution <= 0.0 or track_width_m <= 0.0:
            raise ValueError("wheel geometry and encoder resolution must be positive")
        self.meters_per_count = 2.0 * math.pi * float(wheel_radius_m) / float(counts_per_revolution)
        self.track_width_m = float(track_width_m)
        self.max_dt_sec = max(float(max_dt_sec), 0.0)
        self.linear_scale = float(linear_scale)
        self.angular_scale = float(angular_scale)
        self.angular_sign = 1.0 if angular_sign >= 0.0 else -1.0
        self.wheel_variance_floor_m2 = max(float(wheel_variance_floor_m2), 0.0)
        self.wheel_variance_per_meter = max(float(wheel_variance_per_meter), 0.0)
        self.pose = Pose2D()
        self.covariance = [[0.0] * 3 for _ in range(3)]
        self.last_counts: Optional[Tuple[int, int, int, int]] = None
        self.last_sample_time_sec: Optional[float] = None
        self.last_layout_identity: Optional[Tuple[int, int, int]] = None

    def reset_sample_baseline(self) -> None:
        self.last_counts = None
        self.last_sample_time_sec = None
        self.last_layout_identity = None

    def update(
        self,
        counts: Tuple[int, int, int, int],
        *,
        sample_time_sec: float,
        enabled_mask: int = 0x0F,
        speed_valid_mask: int = 0x0F,
        anomaly_mask: int = 0,
        transport_session_id: int = 0,
        covariance_multiplier: float = 1.0,
        hard_max_speed_mps: float = 0.45,
    ) -> EncoderOdometryUpdate:
        counts = tuple(int(value) for value in counts)
        if len(counts) != 4:
            raise ValueError("exactly four encoder counts are required")
        sample_time = float(sample_time_sec)
        layout = WheelLayout(enabled_mask, speed_valid_mask, anomaly_mask)
        identity = (int(transport_session_id), layout.enabled_mask, layout.eligible_mask)
        if not layout.complete:
            self.last_counts, self.last_sample_time_sec = counts, sample_time
            self.last_layout_identity = identity
            return self._result(0.0, 0.0, 0.0, False, False, 0.0, 0.0)
        if (
            self.last_counts is None
            or self.last_sample_time_sec is None
            or self.last_layout_identity != identity
        ):
            self.last_counts, self.last_sample_time_sec = counts, sample_time
            self.last_layout_identity = identity
            return self._result(0.0, 0.0, 0.0, False, False, 0.0, 0.0)
        dt = sample_time - self.last_sample_time_sec
        previous = self.last_counts
        self.last_counts, self.last_sample_time_sec = counts, sample_time
        if dt <= 0.0 or dt > self.max_dt_sec:
            return self._result(0.0, 0.0, dt, False, False, 0.0, 0.0)
        deltas = tuple(signed_int32_delta(now, old) for now, old in zip(counts, previous))
        max_counts = abs(float(hard_max_speed_mps)) * dt * 1.2 / self.meters_per_count + 2.0
        if any(
            abs(deltas[index]) > max_counts for index in layout.left_indices + layout.right_indices
        ):
            return self._result(0.0, 0.0, dt, False, False, 0.0, 0.0)
        left_counts, right_counts = layout.aggregate(deltas)
        left = left_counts * self.meters_per_count * self.linear_scale
        right = right_counts * self.meters_per_count * self.linear_scale
        ds = 0.5 * (left + right)
        dtheta = (right - left) / self.track_width_m * self.angular_scale * self.angular_sign
        yaw_mid = self.pose.yaw + 0.5 * dtheta
        self.pose = Pose2D(
            self.pose.x + ds * math.cos(yaw_mid),
            self.pose.y + ds * math.sin(yaw_mid),
            math.atan2(math.sin(self.pose.yaw + dtheta), math.cos(self.pose.yaw + dtheta)),
        )
        self._propagate_covariance(left, right, ds, yaw_mid, max(float(covariance_multiplier), 1.0))
        return self._result(ds / dt, dtheta / dt, dt, True, True, left, right)

    def _propagate_covariance(self, left, right, ds, yaw_mid, multiplier):
        f = [
            [1.0, 0.0, -ds * math.sin(yaw_mid)],
            [0.0, 1.0, ds * math.cos(yaw_mid)],
            [0.0, 0.0, 1.0],
        ]
        g = [
            [
                0.5 * math.cos(yaw_mid) + ds * math.sin(yaw_mid) / (2.0 * self.track_width_m),
                0.5 * math.cos(yaw_mid) - ds * math.sin(yaw_mid) / (2.0 * self.track_width_m),
            ],
            [
                0.5 * math.sin(yaw_mid) - ds * math.cos(yaw_mid) / (2.0 * self.track_width_m),
                0.5 * math.sin(yaw_mid) + ds * math.cos(yaw_mid) / (2.0 * self.track_width_m),
            ],
            [-1.0 / self.track_width_m, 1.0 / self.track_width_m],
        ]
        q = [
            (self.wheel_variance_floor_m2 + self.wheel_variance_per_meter * abs(left)) * multiplier,
            (self.wheel_variance_floor_m2 + self.wheel_variance_per_meter * abs(right))
            * multiplier,
        ]
        fp = [
            [sum(f[i][k] * self.covariance[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]
        propagated = [
            [sum(fp[i][k] * f[j][k] for k in range(3)) for j in range(3)] for i in range(3)
        ]
        for i in range(3):
            for j in range(3):
                propagated[i][j] += sum(g[i][k] * q[k] * g[j][k] for k in range(2))
        self.covariance = propagated

    def _result(self, vx, wz, dt, trusted, integrated, left, right):
        flat = tuple(self.covariance[i][j] for i in range(3) for j in range(3))
        return EncoderOdometryUpdate(self.pose, vx, wz, dt, trusted, integrated, left, right, flat)


def covariance_multiplier(
    *,
    wheel_speeds,
    enabled_mask: int = 0x0F,
    speed_valid_mask: int = 0x0F,
    anomaly_mask: int = 0,
    sample_age_sec: float,
    turn_rate: float,
    quality_flags: int = 0,
) -> float:
    speeds = tuple(float(value) for value in wheel_speeds)
    layout = WheelLayout(enabled_mask, speed_valid_mask, anomaly_mask)
    disagreement = layout.pair_disagreement(speeds)
    invalid_wheels = layout.enabled_invalid_count
    return min(
        25.0,
        1.0
        + 4.0 * disagreement
        + 0.5 * abs(float(turn_rate))
        + 4.0 * max(float(sample_age_sec), 0.0)
        + 2.0 * invalid_wheels
        + 3.0 * min(int(quality_flags).bit_count(), 4),
    )
