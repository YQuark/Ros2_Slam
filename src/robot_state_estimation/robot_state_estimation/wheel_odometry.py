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
    degraded: bool
    left_distance: float
    right_distance: float
    pose_covariance: Tuple[float, ...]
    twist_covariance: Tuple[float, float, float, float]


def signed_int32_delta(current: int, previous: int) -> int:
    return ((int(current) - int(previous) + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)


def _se2_functions(theta: float) -> Tuple[float, float, float, float]:
    """Return sinc, cosc and their derivatives with stable small-angle series."""
    value = float(theta)
    value2 = value * value
    if abs(value) < 1.0e-4:
        sinc = 1.0 - value2 / 6.0 + value2 * value2 / 120.0
        cosc = value / 2.0 - value * value2 / 24.0 + value * value2 * value2 / 720.0
        sinc_prime = -value / 3.0 + value * value2 / 30.0
        cosc_prime = 0.5 - value2 / 8.0 + value2 * value2 / 144.0
        return sinc, cosc, sinc_prime, cosc_prime
    sinc = math.sin(value) / value
    cosc = (1.0 - math.cos(value)) / value
    sinc_prime = (value * math.cos(value) - math.sin(value)) / value2
    cosc_prime = (value * math.sin(value) - (1.0 - math.cos(value))) / value2
    return sinc, cosc, sinc_prime, cosc_prime


def se2_increment(distance: float, heading_change: float, yaw: float) -> Tuple[float, float]:
    sinc, cosc, _, _ = _se2_functions(heading_change)
    local_x = float(distance) * sinc
    local_y = float(distance) * cosc
    cos_yaw, sin_yaw = math.cos(float(yaw)), math.sin(float(yaw))
    return (
        cos_yaw * local_x - sin_yaw * local_y,
        sin_yaw * local_x + cos_yaw * local_y,
    )


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
        left_variance_floor_m2: float = 0.000025,
        left_variance_per_meter: float = 0.0025,
        right_variance_floor_m2: Optional[float] = None,
        right_variance_per_meter: Optional[float] = None,
        left_right_correlation: float = 0.0,
        turn_noise_gain: float = 0.0,
        slip_noise_gain: float = 1.0,
        degraded_single_wheel_multiplier: float = 4.0,
    ) -> None:
        if wheel_radius_m <= 0.0 or counts_per_revolution <= 0.0 or track_width_m <= 0.0:
            raise ValueError("wheel geometry and encoder resolution must be positive")
        self.meters_per_count = 2.0 * math.pi * float(wheel_radius_m) / float(counts_per_revolution)
        self.track_width_m = float(track_width_m)
        self.max_dt_sec = max(float(max_dt_sec), 0.0)
        self.linear_scale = float(linear_scale)
        self.angular_scale = float(angular_scale)
        self.angular_sign = 1.0 if angular_sign >= 0.0 else -1.0
        self.left_variance_floor_m2 = max(float(left_variance_floor_m2), 0.0)
        self.left_variance_per_meter = max(float(left_variance_per_meter), 0.0)
        self.right_variance_floor_m2 = max(
            float(
                left_variance_floor_m2
                if right_variance_floor_m2 is None
                else right_variance_floor_m2
            ),
            0.0,
        )
        self.right_variance_per_meter = max(
            float(
                left_variance_per_meter
                if right_variance_per_meter is None
                else right_variance_per_meter
            ),
            0.0,
        )
        self.left_right_correlation = float(left_right_correlation)
        if not -1.0 < self.left_right_correlation < 1.0:
            raise ValueError("left_right_correlation must be strictly between -1 and 1")
        self.turn_noise_gain = max(float(turn_noise_gain), 0.0)
        self.slip_noise_gain = max(float(slip_noise_gain), 0.0)
        self.degraded_single_wheel_multiplier = max(float(degraded_single_wheel_multiplier), 1.0)
        self.pose = Pose2D()
        self.covariance = [[0.0] * 3 for _ in range(3)]
        self.accepted_counts: list[Optional[int]] = [None] * 4
        self.accepted_sample_times: list[Optional[float]] = [None] * 4
        self.last_sample_time_sec: Optional[float] = None
        self.last_layout_identity: Optional[Tuple[int, int, int]] = None

    def reset_sample_baseline(self) -> None:
        self.accepted_counts = [None] * 4
        self.accepted_sample_times = [None] * 4
        self.last_sample_time_sec = None
        self.last_layout_identity = None

    def _establish_baseline(
        self, counts: Tuple[int, int, int, int], sample_time: float, eligible_mask: int
    ) -> None:
        for index in range(4):
            if eligible_mask & (1 << index):
                self.accepted_counts[index] = counts[index]
                self.accepted_sample_times[index] = sample_time
            else:
                self.accepted_counts[index] = None
                self.accepted_sample_times[index] = None
        self.last_sample_time_sec = sample_time

    def update(
        self,
        counts: Tuple[int, int, int, int],
        *,
        sample_time_sec: float,
        enabled_mask: int = 0x0F,
        speed_valid_mask: int = 0x0F,
        anomaly_mask: int = 0,
        transport_session_id: int = 0,
        reset_generation: int = 0,
        covariance_multiplier: float = 1.0,
        hard_max_wheel_peripheral_speed_mps: float = 0.75,
    ) -> EncoderOdometryUpdate:
        converted_counts = tuple(int(value) for value in counts)
        if len(converted_counts) != 4:
            raise ValueError("exactly four encoder counts are required")
        counts = (
            converted_counts[0],
            converted_counts[1],
            converted_counts[2],
            converted_counts[3],
        )
        sample_time = float(sample_time_sec)
        # Upper-v3 speed_valid_mask qualifies the speed estimate only. Encoder
        # counts participate when their motor is enabled and no encoder anomaly
        # is reported by firmware.
        count_layout = WheelLayout(enabled_mask, enabled_mask, anomaly_mask)
        count_eligible_mask = count_layout.eligible_mask
        identity = (
            int(transport_session_id),
            int(reset_generation),
            int(enabled_mask) & 0x0F,
        )
        if self.last_layout_identity != identity or self.last_sample_time_sec is None:
            self.reset_sample_baseline()
            self.last_layout_identity = identity
            self._establish_baseline(counts, sample_time, count_eligible_mask)
            return self._result(0.0, 0.0, 0.0, False, False, False, 0.0, 0.0)
        dt = sample_time - self.last_sample_time_sec
        if dt <= 0.0 or dt > self.max_dt_sec:
            self._establish_baseline(counts, sample_time, count_eligible_mask)
            return self._result(0.0, 0.0, dt, False, False, False, 0.0, 0.0)

        deltas = [0] * 4
        usable_mask = 0
        next_accepted_counts = list(self.accepted_counts)
        next_accepted_times = list(self.accepted_sample_times)
        max_wheel_speed = abs(float(hard_max_wheel_peripheral_speed_mps))
        for index in range(4):
            bit = 1 << index
            if not count_eligible_mask & bit:
                next_accepted_counts[index] = None
                next_accepted_times[index] = None
                continue
            previous_count = self.accepted_counts[index]
            previous_time = self.accepted_sample_times[index]
            if previous_count is None or previous_time != self.last_sample_time_sec:
                next_accepted_counts[index] = counts[index]
                next_accepted_times[index] = sample_time
                continue
            delta = signed_int32_delta(counts[index], previous_count)
            max_counts = max_wheel_speed * dt * 1.2 / self.meters_per_count + 2.0
            if abs(delta) > max_counts:
                # Preserve the last accepted count. The next observation either
                # validates the accumulated delta or explicitly rebaselines.
                continue
            deltas[index] = delta
            usable_mask |= bit
            next_accepted_counts[index] = counts[index]
            next_accepted_times[index] = sample_time

        usable_layout = WheelLayout(enabled_mask, usable_mask, 0)
        degraded = usable_layout.complete and usable_mask != (int(enabled_mask) & 0x0F)
        if not usable_layout.complete:
            # A pose update requires at least one wheel on each side.  Keep the
            # complete accepted baseline unchanged so a short whole-side loss
            # can recover the accumulated displacement on the next valid
            # sample.  Committing only the surviving side would make that
            # interval unrecoverable and bias the next heading update.
            return self._result(0.0, 0.0, dt, False, False, False, 0.0, 0.0)

        self.accepted_counts = next_accepted_counts
        self.accepted_sample_times = next_accepted_times
        self.last_sample_time_sec = sample_time

        left_counts, right_counts = usable_layout.aggregate(deltas)
        left = left_counts * self.meters_per_count * self.linear_scale
        right = right_counts * self.meters_per_count * self.linear_scale
        ds = 0.5 * (left + right)
        dtheta = (right - left) / self.track_width_m * self.angular_scale * self.angular_sign
        previous_yaw = self.pose.yaw
        dx, dy = se2_increment(ds, dtheta, previous_yaw)
        self.pose = Pose2D(
            self.pose.x + dx,
            self.pose.y + dy,
            math.atan2(math.sin(self.pose.yaw + dtheta), math.cos(self.pose.yaw + dtheta)),
        )
        multiplier = max(float(covariance_multiplier), 1.0)
        if degraded:
            multiplier *= self.degraded_single_wheel_multiplier
        q_left, q_right, q_cross = self._wheel_increment_covariance(left, right, multiplier)
        self._propagate_covariance(
            left, right, ds, dtheta, previous_yaw, dx, dy, q_left, q_right, q_cross
        )
        twist_covariance = self._twist_covariance(q_left, q_right, q_cross, dt)
        return self._result(
            ds / dt,
            dtheta / dt,
            dt,
            True,
            True,
            degraded,
            left,
            right,
            twist_covariance,
        )

    def _wheel_increment_covariance(self, left, right, multiplier):
        turn_factor = 1.0 + self.turn_noise_gain * abs(right - left) / self.track_width_m
        quality_factor = 1.0 + self.slip_noise_gain * max(float(multiplier) - 1.0, 0.0)
        q_left = (
            (self.left_variance_floor_m2 + self.left_variance_per_meter * abs(left))
            * turn_factor
            * quality_factor
        )
        q_right = (
            (self.right_variance_floor_m2 + self.right_variance_per_meter * abs(right))
            * turn_factor
            * quality_factor
        )
        q_cross = self.left_right_correlation * math.sqrt(q_left * q_right)
        return q_left, q_right, q_cross

    def _propagate_covariance(
        self,
        left,
        right,
        ds,
        dtheta,
        previous_yaw,
        dx,
        dy,
        q_left,
        q_right,
        q_cross,
    ):
        f = [
            [1.0, 0.0, -dy],
            [0.0, 1.0, dx],
            [0.0, 0.0, 1.0],
        ]
        sinc, cosc, sinc_prime, cosc_prime = _se2_functions(dtheta)
        turn_derivative = self.angular_scale * self.angular_sign / self.track_width_m
        cos_yaw, sin_yaw = math.cos(previous_yaw), math.sin(previous_yaw)

        def column(ds_derivative, theta_derivative):
            d_local_x = ds_derivative * sinc + ds * sinc_prime * theta_derivative
            d_local_y = ds_derivative * cosc + ds * cosc_prime * theta_derivative
            return (
                cos_yaw * d_local_x - sin_yaw * d_local_y,
                sin_yaw * d_local_x + cos_yaw * d_local_y,
                theta_derivative,
            )

        left_column = column(0.5, -turn_derivative)
        right_column = column(0.5, turn_derivative)
        g = [
            [left_column[0], right_column[0]],
            [left_column[1], right_column[1]],
            [left_column[2], right_column[2]],
        ]
        q = [[q_left, q_cross], [q_cross, q_right]]
        fp = [
            [sum(f[i][k] * self.covariance[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]
        propagated = [
            [sum(fp[i][k] * f[j][k] for k in range(3)) for j in range(3)] for i in range(3)
        ]
        for i in range(3):
            for j in range(3):
                propagated[i][j] += sum(
                    g[i][k] * q[k][m] * g[j][m] for k in range(2) for m in range(2)
                )
        self.covariance = propagated

    def _twist_covariance(self, q_left, q_right, q_cross, dt):
        dt2 = max(float(dt) * float(dt), 1e-12)
        summed = q_left + q_right
        turn_scale = self.angular_scale * self.angular_sign / self.track_width_m
        cov_vw = 0.5 * turn_scale * (q_right - q_left) / dt2
        return (
            0.25 * (summed + 2.0 * q_cross) / dt2,
            cov_vw,
            cov_vw,
            turn_scale * turn_scale * (summed - 2.0 * q_cross) / dt2,
        )

    def _result(
        self,
        vx,
        wz,
        dt,
        trusted,
        integrated,
        degraded,
        left,
        right,
        twist_covariance=(0.0, 0.0, 0.0, 0.0),
    ):
        flat = tuple(self.covariance[i][j] for i in range(3) for j in range(3))
        return EncoderOdometryUpdate(
            self.pose,
            vx,
            wz,
            dt,
            trusted,
            integrated,
            degraded,
            left,
            right,
            flat,
            tuple(twist_covariance),
        )


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
    no_redundancy_sides = int(len(layout.left_indices) < 2) + int(len(layout.right_indices) < 2)
    return min(
        25.0,
        1.0
        + 4.0 * disagreement
        + 0.5 * abs(float(turn_rate))
        + 4.0 * max(float(sample_age_sec), 0.0)
        + 2.0 * invalid_wheels
        + 1.0 * no_redundancy_sides
        + 3.0 * min(int(quality_flags).bit_count(), 4),
    )
