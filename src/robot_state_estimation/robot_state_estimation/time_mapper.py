"""MCU sample ordering and bounded MCU-to-ROS clock mapping."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Deque, Optional, Tuple


UINT32_HALF_RANGE = 1 << 31


class SampleDisposition(IntEnum):
    FIRST = 0
    FORWARD = 1
    DUPLICATE = 2
    OUT_OF_ORDER = 3


class SampleOrderTracker:
    def __init__(self) -> None:
        self.session_id: Optional[int] = None
        self.sequence: Optional[int] = None

    def reset(self) -> None:
        self.session_id = None
        self.sequence = None

    def update(self, session_id: int, sequence: int) -> SampleDisposition:
        session = int(session_id) & 0xFFFFFFFFFFFFFFFF
        current = int(sequence) & 0xFFFFFFFF
        if self.session_id != session or self.sequence is None:
            self.session_id, self.sequence = session, current
            return SampleDisposition.FIRST
        delta = (current - self.sequence) & 0xFFFFFFFF
        if delta == 0:
            return SampleDisposition.DUPLICATE
        if delta >= UINT32_HALF_RANGE:
            return SampleDisposition.OUT_OF_ORDER
        self.sequence = current
        return SampleDisposition.FORWARD


@dataclass(frozen=True)
class ClockEstimate:
    sample_ros_sec: float
    sensor_time_sec: float
    scale: float
    offset_sec: float
    residual_p95_sec: float
    stable: bool
    reset: bool


class McuClockMapper:
    """Affine clock fit with uint32 millisecond wrap/restart handling."""

    def __init__(
        self,
        window_size: int = 200,
        min_samples: int = 20,
        max_skew_ppm: float = 1000.0,
        stable_residual_p95_sec: float = 0.010,
    ) -> None:
        self.samples: Deque[Tuple[float, float]] = deque(maxlen=max(int(window_size), 2))
        self.min_samples = max(int(min_samples), 2)
        self.max_skew = abs(float(max_skew_ppm)) * 1e-6
        self.stable_residual_p95_sec = max(float(stable_residual_p95_sec), 0.0)
        self.last_raw_ms: Optional[int] = None
        self.unwrapped_ms: Optional[int] = None
        self.last_ros_sec: Optional[float] = None

    def reset(self) -> None:
        self.samples.clear()
        self.last_raw_ms = None
        self.unwrapped_ms = None
        self.last_ros_sec = None

    def update(self, timestamp_ms: int, receive_ros_sec: float) -> ClockEstimate:
        raw = int(timestamp_ms) & 0xFFFFFFFF
        receive = float(receive_ros_sec)
        if not math.isfinite(receive):
            raise ValueError("receive timestamp is not finite")
        reset = False
        if self.last_raw_ms is None:
            self.unwrapped_ms = raw
        else:
            delta = (raw - self.last_raw_ms) & 0xFFFFFFFF
            if delta > UINT32_HALF_RANGE:
                self.samples.clear()
                self.unwrapped_ms = raw
                reset = True
            else:
                assert self.unwrapped_ms is not None
                self.unwrapped_ms += delta
        self.last_raw_ms = raw
        assert self.unwrapped_ms is not None
        sensor_sec = self.unwrapped_ms * 1e-3
        self.samples.append((sensor_sec, receive))
        scale, offset = self._fit()
        residuals = sorted(abs(ros - (scale * mcu + offset)) for mcu, ros in self.samples)
        p95 = residuals[min(len(residuals) - 1, int(0.95 * len(residuals)))]
        stable = len(self.samples) >= self.min_samples and p95 <= self.stable_residual_p95_sec
        converted = scale * sensor_sec + offset
        sample_ros = converted if stable else receive
        if self.last_ros_sec is not None and sample_ros <= self.last_ros_sec:
            sample_ros = self.last_ros_sec + 1e-9
        self.last_ros_sec = sample_ros
        return ClockEstimate(sample_ros, sensor_sec, scale, offset, p95, stable, reset)

    def _fit(self) -> Tuple[float, float]:
        if len(self.samples) < 2:
            sensor, receive = self.samples[-1]
            return 1.0, receive - sensor
        sample_count = len(self.samples)
        x_mean = math.fsum(item[0] for item in self.samples) / sample_count
        y_mean = math.fsum(item[1] for item in self.samples) / sample_count
        denominator = sum((x - x_mean) ** 2 for x, _ in self.samples)
        scale = (
            1.0
            if denominator <= 1e-12
            else sum((x - x_mean) * (y - y_mean) for x, y in self.samples) / denominator
        )
        scale = max(1.0 - self.max_skew, min(1.0 + self.max_skew, scale))
        offsets = sorted(y - scale * x for x, y in self.samples)
        offset = offsets[min(len(offsets) - 1, max(0, int(0.05 * len(offsets))))]
        return scale, offset
