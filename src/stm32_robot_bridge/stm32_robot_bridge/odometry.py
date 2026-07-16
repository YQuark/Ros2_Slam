"""Pure differential-drive odometry primitives."""

import math
from dataclasses import dataclass
from typing import Optional


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
