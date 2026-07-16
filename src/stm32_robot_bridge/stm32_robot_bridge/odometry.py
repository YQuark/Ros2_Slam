"""Pure differential-drive odometry primitives."""

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class OdometryUpdate:
    pose: Pose2D
    vx: float
    wz: float
    dt: float
    trusted: bool
    integrated: bool


@dataclass(frozen=True)
class CovarianceEstimate:
    linear_variance: float
    angular_variance: float
    pose_xy_variance: float
    pose_yaw_variance: float
    trusted: bool


class DynamicCovarianceModel:
    def __init__(
        self,
        *,
        linear_base_stddev: float = 0.05,
        linear_speed_gain: float = 0.08,
        linear_turn_gain: float = 0.03,
        linear_dt_gain: float = 0.10,
        angular_base_stddev: float = 0.10,
        angular_turn_gain: float = 0.08,
        angular_speed_gain: float = 0.04,
        disagreement_gain: float = 0.30,
    ) -> None:
        self.linear_base_stddev = float(linear_base_stddev)
        self.linear_speed_gain = float(linear_speed_gain)
        self.linear_turn_gain = float(linear_turn_gain)
        self.linear_dt_gain = float(linear_dt_gain)
        self.angular_base_stddev = float(angular_base_stddev)
        self.angular_turn_gain = float(angular_turn_gain)
        self.angular_speed_gain = float(angular_speed_gain)
        self.disagreement_gain = float(disagreement_gain)

    def estimate(self, *, vx, wz, dt, wheel_disagreement, trusted) -> CovarianceEstimate:
        if not trusted:
            return CovarianceEstimate(1000.0, 1000.0, 1.0, math.pi, False)
        vx = abs(float(vx))
        wz = abs(float(wz))
        dt = max(float(dt), 0.0)
        disagreement = abs(float(wheel_disagreement))
        linear_stddev = (
            self.linear_base_stddev
            + self.linear_speed_gain * vx
            + self.linear_turn_gain * wz
            + self.linear_dt_gain * dt
            + 0.5 * self.disagreement_gain * disagreement
        )
        angular_stddev = (
            self.angular_base_stddev
            + self.angular_turn_gain * wz
            + self.angular_speed_gain * vx
            + self.disagreement_gain * disagreement
        )
        linear_variance = linear_stddev**2
        angular_variance = angular_stddev**2
        return CovarianceEstimate(
            linear_variance=linear_variance,
            angular_variance=angular_variance,
            pose_xy_variance=linear_variance * (1.0 + dt),
            pose_yaw_variance=angular_variance * (1.0 + dt),
            trusted=True,
        )


class DifferentialOdometry:
    def __init__(self, max_dt_sec: float = 0.25) -> None:
        self.max_dt_sec = max(float(max_dt_sec), 0.0)
        self.pose = Pose2D()
        self.last_sample_time_sec: Optional[float] = None

    def reset(self, pose: Pose2D = Pose2D()) -> None:
        self.pose = pose
        self.last_sample_time_sec = None

    def reset_sample_baseline(self) -> None:
        self.last_sample_time_sec = None

    def update_from_wheel_speeds(
        self,
        vx: float,
        wz: float,
        *,
        sample_time_sec: float,
        trusted: bool,
    ) -> OdometryUpdate:
        sample_time_sec = float(sample_time_sec)
        vx = float(vx)
        wz = float(wz)
        if self.last_sample_time_sec is None:
            self.last_sample_time_sec = sample_time_sec
            return OdometryUpdate(
                self.pose,
                vx if trusted else 0.0,
                wz if trusted else 0.0,
                0.0,
                bool(trusted),
                False,
            )

        dt = sample_time_sec - self.last_sample_time_sec
        if dt <= 0.0:
            return OdometryUpdate(self.pose, 0.0, 0.0, dt, False, False)

        self.last_sample_time_sec = sample_time_sec
        if dt > self.max_dt_sec or not trusted:
            return OdometryUpdate(self.pose, 0.0, 0.0, dt, False, False)

        self.pose = integrate_midpoint(self.pose, vx, wz, dt)
        return OdometryUpdate(self.pose, vx, wz, dt, True, True)


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
    """Two's-complement modulo delta with half-range ordering."""
    return ((int(current) - int(previous) + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)


class EncoderOdometry:
    """Count-increment odometry with 3x3 Jacobian covariance propagation."""

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
        self.meters_per_count = 2.0 * math.pi * float(wheel_radius_m) / float(counts_per_revolution)
        self.track_width_m = max(float(track_width_m), 1e-6)
        self.max_dt_sec = float(max_dt_sec)
        self.linear_scale = float(linear_scale)
        self.angular_scale = float(angular_scale)
        self.angular_sign = 1.0 if angular_sign >= 0.0 else -1.0
        self.wheel_variance_floor_m2 = float(wheel_variance_floor_m2)
        self.wheel_variance_per_meter = float(wheel_variance_per_meter)
        self.pose = Pose2D()
        self.covariance = [[0.0] * 3 for _ in range(3)]
        self.last_counts: Optional[Tuple[int, int, int, int]] = None
        self.last_sample_time_sec: Optional[float] = None

    def reset_sample_baseline(self) -> None:
        self.last_counts = None
        self.last_sample_time_sec = None

    def update(
        self,
        counts: Tuple[int, int, int, int],
        *,
        sample_time_sec: float,
        valid_mask: int = 0x0F,
        anomaly_mask: int = 0,
        slip_multiplier: float = 1.0,
        hard_max_speed_mps: float = 0.45,
    ) -> EncoderOdometryUpdate:
        counts = (int(counts[0]), int(counts[1]), int(counts[2]), int(counts[3]))
        if self.last_counts is None or self.last_sample_time_sec is None:
            self.last_counts, self.last_sample_time_sec = counts, float(sample_time_sec)
            return self._result(0.0, 0.0, 0.0, False, False, 0.0, 0.0)
        dt = float(sample_time_sec) - self.last_sample_time_sec
        previous = self.last_counts
        self.last_counts, self.last_sample_time_sec = counts, float(sample_time_sec)
        if dt <= 0.0 or dt > self.max_dt_sec:
            return self._result(0.0, 0.0, dt, False, False, 0.0, 0.0)
        deltas = tuple(signed_int32_delta(now, old) for now, old in zip(counts, previous))
        max_counts = (abs(float(hard_max_speed_mps)) * dt * 1.2 / self.meters_per_count) + 2.0
        healthy = [
            bool(valid_mask & (1 << i))
            and not bool(anomaly_mask & (1 << i))
            and abs(deltas[i]) <= max_counts
            for i in range(4)
        ]
        if not all(healthy):
            return self._result(0.0, 0.0, dt, False, False, 0.0, 0.0)
        left = 0.5 * (deltas[0] + deltas[1]) * self.meters_per_count * self.linear_scale
        right = 0.5 * (deltas[2] + deltas[3]) * self.meters_per_count * self.linear_scale
        ds = 0.5 * (left + right)
        dtheta = (right - left) / self.track_width_m * self.angular_scale * self.angular_sign
        yaw_mid = self.pose.yaw + 0.5 * dtheta
        self.pose = Pose2D(
            self.pose.x + ds * math.cos(yaw_mid),
            self.pose.y + ds * math.sin(yaw_mid),
            wrap_angle(self.pose.yaw + dtheta),
        )
        self._propagate_covariance(left, right, ds, dtheta, yaw_mid, slip_multiplier)
        return self._result(ds / dt, dtheta / dt, dt, True, True, left, right)

    def _propagate_covariance(self, left, right, ds, dtheta, yaw_mid, slip_multiplier):
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
            self.wheel_variance_floor_m2 + self.wheel_variance_per_meter * abs(left),
            self.wheel_variance_floor_m2 + self.wheel_variance_per_meter * abs(right),
        ]
        q = [value * max(float(slip_multiplier), 1.0) for value in q]
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


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def apply_deadzone(value: float, threshold: float) -> float:
    if abs(value) < max(threshold, 0.0):
        return 0.0
    return value


def integrate_midpoint(pose: Pose2D, vx: float, wz: float, dt: float) -> Pose2D:
    if dt <= 0.0:
        return pose
    yaw_delta = wz * dt
    yaw_mid = pose.yaw + 0.5 * yaw_delta
    return Pose2D(
        x=pose.x + vx * math.cos(yaw_mid) * dt,
        y=pose.y + vx * math.sin(yaw_mid) * dt,
        yaw=wrap_angle(pose.yaw + yaw_delta),
    )
