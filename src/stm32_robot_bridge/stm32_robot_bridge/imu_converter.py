"""Pure IMU validation and unit-conversion helpers."""

import math
from dataclasses import dataclass
from collections import deque
from statistics import mean
from typing import Deque, Iterable, Optional, Tuple


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
        self.last_timestamp_ms: Optional[int] = None
        self.last_sample_count: Optional[int] = None
        self.unwrapped_timestamp_ms: Optional[int] = None
        self.offset_sec: Optional[float] = None
        self.last_sample_ros_sec: Optional[float] = None
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

        assert self.last_sample_count is not None
        assert self.unwrapped_timestamp_ms is not None
        assert self.offset_sec is not None
        assert self.last_sample_ros_sec is not None

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


IMU_QUALITY_GYRO_INVALID = 1 << 0
IMU_QUALITY_ACCEL_INVALID = 1 << 1
IMU_QUALITY_ORIENTATION_INVALID = 1 << 2
IMU_QUALITY_TIMESTAMP_INVALID = 1 << 3
IMU_QUALITY_GYRO_WARNING = 1 << 4
IMU_QUALITY_ACCEL_WARNING = 1 << 5
IMU_QUALITY_ORIENTATION_WARNING = 1 << 6
IMU_QUALITY_TIMESTAMP_WARNING = 1 << 7
IMU_QUALITY_FATAL = 1 << 8
IMU_QUALITY_SENSOR_RESET = 1 << 9


@dataclass(frozen=True)
class ImuFieldValidity:
    gyro_valid: bool
    accel_valid: bool
    orientation_valid: bool
    timestamp_valid: bool
    warning: bool


def classify_imu_quality(quality_flags: int, *, online: bool, error: bool) -> ImuFieldValidity:
    flags = int(quality_flags)
    base_valid = online and not error and not bool(flags & IMU_QUALITY_FATAL)
    return ImuFieldValidity(
        gyro_valid=base_valid and not bool(flags & IMU_QUALITY_GYRO_INVALID),
        accel_valid=base_valid and not bool(flags & IMU_QUALITY_ACCEL_INVALID),
        orientation_valid=base_valid and not bool(flags & IMU_QUALITY_ORIENTATION_INVALID),
        timestamp_valid=base_valid and not bool(flags & IMU_QUALITY_TIMESTAMP_INVALID),
        warning=bool(
            flags
            & (
                IMU_QUALITY_GYRO_WARNING
                | IMU_QUALITY_ACCEL_WARNING
                | IMU_QUALITY_ORIENTATION_WARNING
                | IMU_QUALITY_TIMESTAMP_WARNING
                | IMU_QUALITY_SENSOR_RESET
            )
        ),
    )


@dataclass(frozen=True)
class AffineClockEstimate:
    sample_ros_sec: float
    scale: float
    offset_sec: float
    residual_mean_sec: float
    residual_p95_sec: float
    stable: bool
    reset: bool


class AffineClockSynchronizer:
    """Bounded least-squares MCU clock mapping with wrap/restart handling."""

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
        self.stable_residual_p95_sec = float(stable_residual_p95_sec)
        self.last_raw_ms: Optional[int] = None
        self.unwrapped_ms: Optional[int] = None
        self.last_ros_sec: Optional[float] = None
        self.wrap_count = 0
        self.reset_count = 0

    def reset(self) -> None:
        self.samples.clear()
        self.last_raw_ms = self.unwrapped_ms = None
        self.last_ros_sec = None

    def update(self, timestamp_ms: int, receive_ros_sec: float) -> AffineClockEstimate:
        raw = int(timestamp_ms) & 0xFFFFFFFF
        receive = float(receive_ros_sec)
        if not math.isfinite(receive):
            raise InvalidImuSample("receive timestamp is not finite")
        reset = False
        if self.last_raw_ms is None:
            self.unwrapped_ms = raw
        else:
            delta = (raw - self.last_raw_ms) & 0xFFFFFFFF
            if delta > UINT32_HALF_RANGE:
                self.reset_count += 1
                self.samples.clear()
                self.unwrapped_ms = raw
                reset = True
            else:
                if raw < self.last_raw_ms:
                    self.wrap_count += 1
                assert self.unwrapped_ms is not None
                self.unwrapped_ms += delta
        self.last_raw_ms = raw
        assert self.unwrapped_ms is not None
        sensor = float(self.unwrapped_ms) * 1e-3
        self.samples.append((sensor, receive))
        scale, offset = self._fit()
        residuals = [ros - (scale * mcu + offset) for mcu, ros in self.samples]
        ordered = sorted(abs(value) for value in residuals)
        p95 = ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))] if ordered else 0.0
        converted = scale * sensor + offset
        stable = len(self.samples) >= self.min_samples and p95 <= self.stable_residual_p95_sec
        sample_ros = converted if stable else receive
        if self.last_ros_sec is not None and sample_ros <= self.last_ros_sec:
            sample_ros = self.last_ros_sec + 1e-9
        self.last_ros_sec = sample_ros
        return AffineClockEstimate(
            sample_ros, scale, offset, mean(residuals) if residuals else 0.0, p95, stable, reset
        )

    def _fit(self) -> Tuple[float, float]:
        if len(self.samples) < 2:
            sensor, receive = self.samples[-1]
            return 1.0, receive - sensor
        x_mean = mean(value[0] for value in self.samples)
        y_mean = mean(value[1] for value in self.samples)
        denominator = sum((x - x_mean) ** 2 for x, _ in self.samples)
        scale = (
            1.0
            if denominator <= 1e-12
            else sum((x - x_mean) * (y - y_mean) for x, y in self.samples) / denominator
        )
        scale = max(1.0 - self.max_skew, min(1.0 + self.max_skew, scale))
        # Use the lower residual envelope so serial queueing delay does not bias sample time late.
        offsets = sorted(y - scale * x for x, y in self.samples)
        offset = offsets[min(len(offsets) - 1, max(0, int(0.05 * len(offsets))))]
        return scale, offset

    @property
    def scale(self) -> float:
        return self._fit()[0] if self.samples else 1.0

    @property
    def residual_p95_sec(self) -> float:
        if not self.samples:
            return 0.0
        scale, offset = self._fit()
        residuals = sorted(abs(ros - (scale * mcu + offset)) for mcu, ros in self.samples)
        return residuals[min(len(residuals) - 1, int(0.95 * len(residuals)))]


def normalize_quaternion(values: Iterable[float]) -> Optional[Quaternion]:
    converted = tuple(float(value) for value in values)
    if len(converted) != 4 or not all(math.isfinite(value) for value in converted):
        return None
    norm = math.sqrt(sum(value * value for value in converted))
    if norm <= 1e-9:
        return None
    x, y, z, w = converted
    return x / norm, y / norm, z / norm, w / norm


def gyro_dps_to_rad(values: Iterable[float]) -> Vector3:
    x, y, z = (float(value) for value in values)
    return math.radians(x), math.radians(y), math.radians(z)


def accel_g_to_mps2(values: Iterable[float]) -> Vector3:
    x, y, z = (float(value) for value in values)
    return x * GRAVITY_MPS2, y * GRAVITY_MPS2, z * GRAVITY_MPS2
