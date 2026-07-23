"""ROS-independent deterministic four-wheel fake base model."""

import math
from dataclasses import dataclass
from typing import Tuple

from robot_chassis_model.wheel_layout import DEFAULT_ENABLED_MASK


def signed_int32(value: int) -> int:
    return ((int(value) + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)


@dataclass(frozen=True)
class FakeSample:
    wheel_speeds: Tuple[float, float, float, float]
    wheel_targets: Tuple[float, float, float, float]
    encoder_counts: Tuple[int, int, int, int]
    vx: float
    wz: float


class StatusSampleLatch:
    """Repeat a STATUS sample without inventing a new sequence or payload."""

    def __init__(self) -> None:
        self.sequence = 0
        self.snapshot = None

    def update(self, snapshot, *, repeat: bool):
        if not repeat or self.snapshot is None:
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            self.snapshot = snapshot
        return self.sequence, self.snapshot


class FakeBaseModel:
    def __init__(
        self,
        *,
        wheel_radius_m: float,
        track_width_m: float,
        counts_per_revolution: float,
        response_tau_sec: float = 0.10,
        enabled_mask: int = DEFAULT_ENABLED_MASK,
    ) -> None:
        self.track_width_m = float(track_width_m)
        self.meters_per_count = 2.0 * math.pi * float(wheel_radius_m) / float(counts_per_revolution)
        self.response_tau_sec = max(float(response_tau_sec), 1e-6)
        self.target_vx = self.target_wz = 0.0
        self.vx = self.wz = 0.0
        self._count_float = [0.0] * 4
        self.enabled_mask = int(enabled_mask) & 0x0F

    def set_target(self, vx: float, wz: float) -> None:
        self.target_vx, self.target_wz = float(vx), float(wz)

    def release(self) -> None:
        self.target_vx = self.target_wz = 0.0

    def step(self, dt: float, *, slip_scale: float = 1.0) -> FakeSample:
        dt = max(float(dt), 0.0)
        alpha = 1.0 - math.exp(-dt / self.response_tau_sec)
        self.vx += alpha * (self.target_vx - self.vx)
        self.wz += alpha * (self.target_wz - self.wz)
        target_left = self.target_vx - 0.5 * self.target_wz * self.track_width_m
        target_right = self.target_vx + 0.5 * self.target_wz * self.track_width_m
        left = (self.vx - 0.5 * self.wz * self.track_width_m) * float(slip_scale)
        right = (self.vx + 0.5 * self.wz * self.track_width_m) * float(slip_scale)
        all_speeds = (left, left, right, right)
        all_targets = (target_left, target_left, target_right, target_right)
        speeds = tuple(
            value if self.enabled_mask & (1 << index) else 0.0
            for index, value in enumerate(all_speeds)
        )
        targets = tuple(
            value if self.enabled_mask & (1 << index) else 0.0
            for index, value in enumerate(all_targets)
        )
        for index, speed in enumerate(speeds):
            self._count_float[index] += speed * dt / self.meters_per_count
        counts = tuple(signed_int32(round(value)) for value in self._count_float)
        return FakeSample(speeds, targets, counts, self.vx, self.wz)
