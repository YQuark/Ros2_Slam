"""Pure IMU validation and unit-conversion helpers."""

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


GRAVITY_MPS2 = 9.80665
Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
UINT32_MODULUS = 1 << 32
UINT32_HALF_RANGE = 1 << 31


class InvalidImuSample(ValueError):
    pass


@dataclass(frozen=True)
class ImuTiming:
    sample_ros_sec: float
    dropped_samples: int
    reset: bool


class ImuClockSynchronizer:
    def __init__(self, offset_alpha: float = 0.02) -> None:
        self.offset_alpha = min(max(float(offset_alpha), 0.0), 1.0)
        self.last_timestamp_ms = None
        self.last_sample_count = None
        self.unwrapped_timestamp_ms = None
        self.offset_sec = None
        self.last_sample_ros_sec = None
        self.total_dropped_samples = 0

    def reset(self) -> None:
        self.last_timestamp_ms = None
        self.last_sample_count = None
        self.unwrapped_timestamp_ms = None
        self.offset_sec = None
        self.last_sample_ros_sec = None

    def update(
        self,
        *,
        timestamp_ms: int,
        sample_count: int,
        receive_ros_sec: float,
    ) -> ImuTiming:
        timestamp_ms = int(timestamp_ms) & 0xFFFFFFFF
        sample_count = int(sample_count) & 0xFFFFFFFF
        receive_ros_sec = float(receive_ros_sec)
        if not math.isfinite(receive_ros_sec):
            raise InvalidImuSample("receive timestamp is not finite")

        if self.last_timestamp_ms is None:
            return self._initialize(timestamp_ms, sample_count, receive_ros_sec, reset=False)

        sample_delta = (sample_count - self.last_sample_count) & 0xFFFFFFFF
        if sample_delta == 0:
            raise InvalidImuSample("duplicate IMU sample_count")
        timestamp_delta = (timestamp_ms - self.last_timestamp_ms) & 0xFFFFFFFF
        reset_detected = sample_delta > UINT32_HALF_RANGE or timestamp_delta > UINT32_HALF_RANGE
        if reset_detected:
            return self._initialize(timestamp_ms, sample_count, receive_ros_sec, reset=True)
        if timestamp_delta == 0:
            raise InvalidImuSample("IMU timestamp did not advance")

        dropped = max(sample_delta - 1, 0)
        self.total_dropped_samples += dropped
        self.unwrapped_timestamp_ms += timestamp_delta
        sensor_sec = self.unwrapped_timestamp_ms * 1e-3
        observed_offset = receive_ros_sec - sensor_sec
        self.offset_sec += self.offset_alpha * (observed_offset - self.offset_sec)
        sample_ros_sec = sensor_sec + self.offset_sec
        if sample_ros_sec <= self.last_sample_ros_sec:
            raise InvalidImuSample("converted IMU timestamp is not monotonic")

        self.last_timestamp_ms = timestamp_ms
        self.last_sample_count = sample_count
        self.last_sample_ros_sec = sample_ros_sec
        return ImuTiming(sample_ros_sec, dropped, False)

    def _initialize(
        self,
        timestamp_ms: int,
        sample_count: int,
        receive_ros_sec: float,
        *,
        reset: bool,
    ) -> ImuTiming:
        self.last_timestamp_ms = timestamp_ms
        self.last_sample_count = sample_count
        self.unwrapped_timestamp_ms = timestamp_ms
        self.offset_sec = receive_ros_sec - timestamp_ms * 1e-3
        self.last_sample_ros_sec = receive_ros_sec
        return ImuTiming(receive_ros_sec, 0, reset)


def normalize_quaternion(values: Iterable[float]) -> Optional[Quaternion]:
    converted = tuple(float(value) for value in values)
    if len(converted) != 4 or not all(math.isfinite(value) for value in converted):
        return None
    norm = math.sqrt(sum(value * value for value in converted))
    if norm <= 1e-9:
        return None
    return tuple(value / norm for value in converted)


def gyro_dps_to_rad(values: Iterable[float]) -> Vector3:
    x, y, z = (float(value) for value in values)
    return math.radians(x), math.radians(y), math.radians(z)


def accel_g_to_mps2(values: Iterable[float]) -> Vector3:
    x, y, z = (float(value) for value in values)
    return x * GRAVITY_MPS2, y * GRAVITY_MPS2, z * GRAVITY_MPS2
