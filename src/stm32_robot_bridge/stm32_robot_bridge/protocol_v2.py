import struct
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

FRAME_SOF = 0xA5
FRAME_SOF2 = 0x5A

PROTOCOL_VERSION = 2
MAX_PAYLOAD_SIZE = 99
MAX_LENGTH = MAX_PAYLOAD_SIZE + 1

CMD_SET_VELOCITY = 0x01
CMD_ESTOP = 0x02
CMD_LINE_CTRL = 0x03
CMD_CLEAR_FAULT = 0x04
CMD_STATUS = 0x81
CMD_DIAGNOSTIC = 0x82
CMD_IMU_STATUS = 0x83

VELOCITY_PAYLOAD_SIZE = 10
ESTOP_PAYLOAD_SIZE = 1
LINE_CTRL_PAYLOAD_SIZE = 1
CLEAR_FAULT_PAYLOAD_SIZE = 0
STATUS_PAYLOAD_SIZE = 65
DIAGNOSTIC_PAYLOAD_SIZE = 28
IMU_STATUS_PAYLOAD_SIZE = 99
MODE_UPPER_DEFAULT = 2

STATUS_FLAG_ESTOP = 1 << 0
STATUS_FLAG_FAULT_STOP = 1 << 1
STATUS_FLAG_LINE_ENABLED = 1 << 2
STATUS_FLAG_SPEED_VALID_ALL = 1 << 3

IMU_FLAG_ONLINE = 1 << 0
IMU_FLAG_CALIBRATED = 1 << 1
IMU_FLAG_ERROR = 1 << 2
IMU_FLAG_SENSOR_TIME = 1 << 3

LEFT_MOTOR_MASK = 0b0011
RIGHT_MOTOR_MASK = 0b1100


@dataclass(frozen=True)
class StatusPayload:
    version: int
    status_flags: int
    control_source: int
    motor_enabled_mask: int
    error_flags: int
    latched_error_flags: int
    battery_voltage: float
    motor_speed_mps: Tuple[float, float, float, float]
    encoder_count: Tuple[int, int, int, int]
    motor_current_a: Tuple[float, float, float, float]
    motor_target_mps: Tuple[float, float, float, float]
    motor_output_permille: Tuple[int, int, int, int]
    motor_speed_valid_mask: int
    encoder_anomaly_mask: int
    comm_health_flags: int


@dataclass(frozen=True)
class ImuStatusPayload:
    version: int
    accel_g: Tuple[float, float, float]
    gyro_corrected_dps: Tuple[float, float, float]
    euler_deg: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]  # (x, y, z, w) in ROS convention, remapped from wire (w, x, y, z)
    timestamp_ms: int
    sensor_time: int
    sample_count: int
    quality_flags: int
    quality_counters: Tuple[int, int, int, int, int, int, int]
    status_flags: int
    temperature_c: int


@dataclass(frozen=True)
class DiagnosticPayload:
    """STM32 DIAGNOSTIC frame (0x82, 28B payload, reported at 200ms).

    Fields match upper_diagnostic_payload_t in upper_protocol.h.
    """
    protocol_version: int
    schema_version: int
    post_done: int
    imu_status_flags: int
    post_error_flags: int
    adc_invalid_reason_flags: int
    task_timeout_mask: int
    imu_quality_flags: int
    reset_reason_flags: int
    uptime_ms: int


@dataclass(frozen=True)
class AggregatedStatus:
    odom_trusted: bool
    left_speed_mps: float
    right_speed_mps: float
    left_target_mps: float
    right_target_mps: float
    left_encoder_count: int
    right_encoder_count: int
    left_current_a: float
    right_current_a: float
    vx_mps: float
    wz_radps: float


# CRC-8 lookup table from STM32F407 firmware (upper_protocol.c).
# Must be bit-identical to the firmware table for frame CRC validation.
_CRC8_TABLE = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
]


def crc8(data: bytes) -> int:
    """Compute CRC-8 using the STM32 firmware's lookup table."""
    crc = 0
    for byte in data:
        crc = _CRC8_TABLE[crc ^ byte]
    return crc


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload too large: {len(payload)} > {MAX_PAYLOAD_SIZE}")
    length = len(payload) + 1
    body = bytes([length, cmd]) + payload
    return bytes([FRAME_SOF, FRAME_SOF2]) + body + bytes([crc8(body)])


def encode_velocity_payload(vx: float, wz: float, enable: bool, mode: int = MODE_UPPER_DEFAULT) -> bytes:
    return struct.pack("<ffBB", float(vx), float(wz), 1 if enable else 0, int(mode) & 0xFF)


def encode_estop_payload(enabled: bool) -> bytes:
    return bytes([1 if enabled else 0])


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> List[Tuple[int, bytes]]:
        self._buffer.extend(chunk)
        frames: List[Tuple[int, bytes]] = []

        while True:
            header_index = self._find_header()
            if header_index < 0:
                self._retain_possible_header_tail()
                break
            if header_index > 0:
                del self._buffer[:header_index]
            if len(self._buffer) < 3:
                break

            length = self._buffer[2]
            if length == 0 or length > MAX_LENGTH:
                del self._buffer[0]
                continue

            frame_len = length + 4
            if len(self._buffer) < frame_len:
                break

            body = bytes(self._buffer[2:3 + length])
            received_crc = self._buffer[frame_len - 1]
            if crc8(body) != received_crc:
                del self._buffer[0]
                continue

            cmd = self._buffer[3]
            payload = bytes(self._buffer[4:3 + length])
            frames.append((cmd, payload))
            del self._buffer[:frame_len]

        return frames

    def _find_header(self) -> int:
        for index in range(max(0, len(self._buffer) - 1)):
            if self._buffer[index] == FRAME_SOF and self._buffer[index + 1] == FRAME_SOF2:
                return index
        return -1

    def _retain_possible_header_tail(self) -> None:
        if self._buffer and self._buffer[-1] == FRAME_SOF:
            del self._buffer[:-1]
        else:
            self._buffer.clear()


def decode_status_payload(payload: bytes) -> Optional[StatusPayload]:
    if len(payload) != STATUS_PAYLOAD_SIZE:
        return None
    if payload[0] != PROTOCOL_VERSION:
        return None

    speeds_raw = struct.unpack_from("<4h", payload, 14)
    encoders = struct.unpack_from("<4i", payload, 22)
    currents_raw = struct.unpack_from("<4H", payload, 38)
    targets_raw = struct.unpack_from("<4h", payload, 46)
    outputs = struct.unpack_from("<4h", payload, 54)

    return StatusPayload(
        version=payload[0],
        status_flags=payload[1],
        control_source=payload[2],
        motor_enabled_mask=payload[3],
        error_flags=struct.unpack_from("<I", payload, 4)[0],
        latched_error_flags=struct.unpack_from("<I", payload, 8)[0],
        battery_voltage=struct.unpack_from("<H", payload, 12)[0] / 1000.0,
        motor_speed_mps=tuple(value / 1000.0 for value in speeds_raw),
        encoder_count=tuple(encoders),
        motor_current_a=tuple(value / 1000.0 for value in currents_raw),
        motor_target_mps=tuple(value / 1000.0 for value in targets_raw),
        motor_output_permille=tuple(outputs),
        motor_speed_valid_mask=payload[62],
        encoder_anomaly_mask=payload[63],
        comm_health_flags=payload[64],
    )


def decode_imu_status_payload(payload: bytes) -> Optional[ImuStatusPayload]:
    if len(payload) != IMU_STATUS_PAYLOAD_SIZE:
        return None
    if payload[0] != PROTOCOL_VERSION:
        return None

    offset = 1
    accel = struct.unpack_from("<3f", payload, offset)
    offset += 12
    gyro = struct.unpack_from("<3f", payload, offset)
    offset += 12
    euler = struct.unpack_from("<3f", payload, offset)
    offset += 12
    # STM32 wire format: [w, x, y, z]; remap to ROS convention [x, y, z, w]
    w, x, y, z = struct.unpack_from("<4f", payload, offset)
    quaternion = (x, y, z, w)
    offset += 16
    timestamp_ms = struct.unpack_from("<I", payload, offset)[0]
    offset += 4
    sensor_time = struct.unpack_from("<I", payload, offset)[0]
    offset += 4
    sample_count = struct.unpack_from("<I", payload, offset)[0]
    offset += 4
    quality_flags = struct.unpack_from("<I", payload, offset)[0]
    offset += 4
    quality_counters = struct.unpack_from("<7I", payload, offset)
    offset += 28

    return ImuStatusPayload(
        version=payload[0],
        accel_g=tuple(accel),
        gyro_corrected_dps=tuple(gyro),
        euler_deg=tuple(euler),
        quaternion=tuple(quaternion),
        timestamp_ms=timestamp_ms,
        sensor_time=sensor_time,
        sample_count=sample_count,
        quality_flags=quality_flags,
        quality_counters=tuple(quality_counters),
        status_flags=payload[offset],
        temperature_c=struct.unpack_from("<b", payload, offset + 1)[0],
    )


def decode_diagnostic_payload(payload: bytes) -> Optional[DiagnosticPayload]:
    if len(payload) != DIAGNOSTIC_PAYLOAD_SIZE:
        return None
    if payload[0] != PROTOCOL_VERSION:
        return None

    return DiagnosticPayload(
        protocol_version=payload[0],
        schema_version=payload[1],
        post_done=payload[2],
        imu_status_flags=payload[3],
        post_error_flags=struct.unpack_from("<I", payload, 4)[0],
        adc_invalid_reason_flags=struct.unpack_from("<I", payload, 8)[0],
        task_timeout_mask=struct.unpack_from("<H", payload, 12)[0],
        imu_quality_flags=struct.unpack_from("<I", payload, 16)[0],
        reset_reason_flags=struct.unpack_from("<I", payload, 20)[0],
        uptime_ms=struct.unpack_from("<I", payload, 24)[0],
    )


def encode_line_ctrl_payload(enabled: bool) -> bytes:
    return bytes([1 if enabled else 0])


def encode_clear_fault_payload() -> bytes:
    return b""


def aggregate_status(status: StatusPayload, wheel_track_width: float, drive_mode: str = "differential") -> AggregatedStatus:
    left_indices = _enabled_indices(status.motor_enabled_mask & LEFT_MOTOR_MASK)
    right_indices = _enabled_indices(status.motor_enabled_mask & RIGHT_MOTOR_MASK)

    left_present = len(left_indices) > 0
    right_present = len(right_indices) > 0
    speed_valid = all((status.motor_speed_valid_mask & (1 << index)) != 0 for index in left_indices + right_indices)
    odom_trusted = left_present and right_present and speed_valid

    left_speed = _average(status.motor_speed_mps, left_indices)
    right_speed = _average(status.motor_speed_mps, right_indices)
    left_target = _average(status.motor_target_mps, left_indices)
    right_target = _average(status.motor_target_mps, right_indices)
    left_encoder = int(round(_average(status.encoder_count, left_indices)))
    right_encoder = int(round(_average(status.encoder_count, right_indices)))
    left_current = _sum(status.motor_current_a, left_indices)
    right_current = _sum(status.motor_current_a, right_indices)

    if drive_mode == "differential":
        if odom_trusted:
            vx = (left_speed + right_speed) / 2.0
            wz = (right_speed - left_speed) / max(float(wheel_track_width), 1e-6)
        else:
            vx = 0.0
            wz = 0.0
    else:
        raise ValueError(f"Unknown drive_mode: {drive_mode!r}")
    return AggregatedStatus(
        odom_trusted=odom_trusted,
        left_speed_mps=left_speed,
        right_speed_mps=right_speed,
        left_target_mps=left_target,
        right_target_mps=right_target,
        left_encoder_count=left_encoder,
        right_encoder_count=right_encoder,
        left_current_a=left_current,
        right_current_a=right_current,
        vx_mps=vx,
        wz_radps=wz,
    )


class CommandStream:
    def __init__(self, cmd_timeout_sec: float, keepalive_sec: float, mode: int = MODE_UPPER_DEFAULT) -> None:
        self.cmd_timeout_sec = max(float(cmd_timeout_sec), 0.0)
        self.keepalive_sec = max(float(keepalive_sec), 0.0)
        self.mode = int(mode) & 0xFF
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.last_command_time: Optional[float] = None
        self.last_send_time: Optional[float] = None
        self.last_payload: Optional[bytes] = None
        self.upper_owned = False
        self.release_sent = True

    def update_command(self, vx: float, wz: float, now_sec: float) -> None:
        self.target_vx = float(vx)
        self.target_wz = float(wz)
        self.last_command_time = float(now_sec)
        self.release_sent = False

    def tick(self, now_sec: float, send_payload: Callable[[bytes], object]) -> bool:
        now_sec = float(now_sec)
        if self.last_command_time is None:
            return False

        active = (now_sec - self.last_command_time) <= self.cmd_timeout_sec
        if active:
            payload = encode_velocity_payload(self.target_vx, self.target_wz, True, self.mode)
            due = (
                self.last_payload != payload
                or self.last_send_time is None
                or (now_sec - self.last_send_time) >= max(self.keepalive_sec, 0.001)
            )
            if not due:
                return False
            if send_payload(payload) is False:
                return False
            self.last_payload = payload
            self.last_send_time = now_sec
            self.upper_owned = True
            self.release_sent = False
            return True

        if self.upper_owned and not self.release_sent:
            return self.release(send_payload, now_sec)
        return False

    def release(self, send_payload: Callable[[bytes], object], now_sec: Optional[float] = None) -> bool:
        payload = encode_velocity_payload(0.0, 0.0, False, self.mode)
        if send_payload(payload) is False:
            return False
        self.last_payload = payload
        self.last_send_time = float(now_sec) if now_sec is not None else None
        self.upper_owned = False
        self.release_sent = True
        return True


def _enabled_indices(mask: int) -> List[int]:
    return [index for index in range(4) if (mask & (1 << index)) != 0]


def _average(values: Tuple[float, ...], indices: List[int]) -> float:
    if not indices:
        return 0.0
    return sum(values[index] for index in indices) / len(indices)


def _sum(values: Tuple[float, ...], indices: List[int]) -> float:
    return sum(values[index] for index in indices)