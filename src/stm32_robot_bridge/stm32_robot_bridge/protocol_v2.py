import struct
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

FRAME_SOF = 0xA5
FRAME_SOF2 = 0x5A

PROTOCOL_VERSION = 2
MAX_PAYLOAD_SIZE = 64
MAX_LENGTH = MAX_PAYLOAD_SIZE + 1

CMD_SET_VELOCITY = 0x01
CMD_ESTOP = 0x02
CMD_LINE_CTRL = 0x03
CMD_STATUS = 0x81

VELOCITY_PAYLOAD_SIZE = 10
ESTOP_PAYLOAD_SIZE = 1
STATUS_PAYLOAD_SIZE = 64
MODE_UPPER_DEFAULT = 2

STATUS_FLAG_ESTOP = 1 << 0
STATUS_FLAG_FAULT_STOP = 1 << 1
STATUS_FLAG_LINE_ENABLED = 1 << 2
STATUS_FLAG_SPEED_VALID_ALL = 1 << 3

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


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x5E) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
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
    )


def aggregate_status(status: StatusPayload, wheel_track_width: float) -> AggregatedStatus:
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

    if odom_trusted:
        vx = (left_speed + right_speed) / 2.0
        wz = (right_speed - left_speed) / max(float(wheel_track_width), 1e-6)
    else:
        vx = 0.0
        wz = 0.0

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
