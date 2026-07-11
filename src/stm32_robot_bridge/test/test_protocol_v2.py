import math
import struct

import pytest

from stm32_robot_bridge.protocol_v2 import (
    CMD_CLEAR_FAULT,
    CMD_DIAGNOSTIC,
    CMD_ESTOP,
    CMD_IMU_STATUS,
    CMD_LINE_CTRL,
    CMD_SET_VELOCITY,
    CMD_STATUS,
    DIAGNOSTIC_PAYLOAD_SIZE,
    IMU_STATUS_PAYLOAD_SIZE,
    MAX_PAYLOAD_SIZE,
    PROTOCOL_VERSION,
    STATUS_FLAG_ESTOP,
    STATUS_FLAG_FAULT_STOP,
    STATUS_FLAG_SPEED_VALID_ALL,
    STATUS_PAYLOAD_SIZE,
    CommandStream,
    FrameParser,
    aggregate_status,
    build_frame,
    crc8,
    decode_diagnostic_payload,
    decode_imu_status_payload,
    decode_status_payload,
    encode_clear_fault_payload,
    encode_estop_payload,
    encode_line_ctrl_payload,
    encode_velocity_payload,
)


# CRC-8 table from STM32 firmware (must match protocol_v2._CRC8_TABLE)
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


def independent_crc8(data: bytes) -> int:
    """CRC-8 using the same table as the STM32 firmware."""
    crc = 0
    for byte in data:
        crc = _CRC8_TABLE[crc ^ byte]
    return crc


def make_status_payload(
    *,
    version=PROTOCOL_VERSION,
    status_flags=0x08,
    control_source=1,
    enabled_mask=0b0110,
    error_flags=0x11223344,
    latched_error_flags=0x55667788,
    battery_mv=12345,
    speeds=(0, 120, 340, 0),
    encoders=(10, 2000, -3000, 40),
    currents=(0, 111, 222, 0),
    targets=(0, 130, 350, 0),
    outputs=(0, 501, -502, 0),
    speed_valid_mask=0b0110,
    encoder_anomaly_mask=0xA5,
    comm_health_flags=0x5A,
) -> bytes:
    payload = bytearray(STATUS_PAYLOAD_SIZE)
    payload[0] = version
    payload[1] = status_flags
    payload[2] = control_source
    payload[3] = enabled_mask
    struct.pack_into("<I", payload, 4, error_flags)
    struct.pack_into("<I", payload, 8, latched_error_flags)
    struct.pack_into("<H", payload, 12, battery_mv)
    struct.pack_into("<4h", payload, 14, *speeds)
    struct.pack_into("<4i", payload, 22, *encoders)
    struct.pack_into("<4H", payload, 38, *currents)
    struct.pack_into("<4h", payload, 46, *targets)
    struct.pack_into("<4h", payload, 54, *outputs)
    payload[62] = speed_valid_mask
    payload[63] = encoder_anomaly_mask
    payload[64] = comm_health_flags
    return bytes(payload)


def make_imu_status_payload(
    *,
    version=PROTOCOL_VERSION,
    accel=(0.1, -0.2, 0.3),
    gyro=(1.5, -2.5, 3.5),
    euler=(10.0, -20.0, 30.0),
    quaternion=(0.5, 0.5, -0.5, 0.5),  # wire order [w, x, y, z]; decode remaps to [x, y, z, w]
    timestamp_ms=123456,
    sensor_time=654321,
    sample_count=42,
    quality_flags=0x11223344,
    quality_counters=(1, 2, 3, 4, 5, 6, 7),
    status_flags=0x0B,
    temperature_c=-12,
) -> bytes:
    payload = bytearray(IMU_STATUS_PAYLOAD_SIZE)
    payload[0] = version
    offset = 1
    for values in (accel, gyro, euler, quaternion):
        for value in values:
            struct.pack_into("<f", payload, offset, value)
            offset += 4
    struct.pack_into("<I", payload, offset, timestamp_ms)
    offset += 4
    struct.pack_into("<I", payload, offset, sensor_time)
    offset += 4
    struct.pack_into("<I", payload, offset, sample_count)
    offset += 4
    struct.pack_into("<I", payload, offset, quality_flags)
    offset += 4
    struct.pack_into("<7I", payload, offset, *quality_counters)
    offset += 28
    payload[offset] = status_flags
    payload[offset + 1] = temperature_c & 0xFF
    return bytes(payload)


def test_set_velocity_frame_uses_cmd_length_crc8_and_little_endian_payload():
    payload = encode_velocity_payload(0.125, -0.75, enable=True, mode=2)
    frame = build_frame(CMD_SET_VELOCITY, payload)

    assert frame[:2] == b"\xA5\x5A"
    assert frame[2] == 11
    assert frame[3] == CMD_SET_VELOCITY
    assert len(payload) == 10
    assert payload == struct.pack("<ffBB", 0.125, -0.75, 1, 2)
    assert frame[-1] == independent_crc8(frame[2:-1])


def test_frame_parser_accepts_complete_split_sticky_and_garbage_prefixed_status_frames():
    payload = make_status_payload()
    frame = build_frame(CMD_STATUS, payload)
    parser = FrameParser()

    assert parser.feed(frame[:13]) == []
    assert parser.feed(frame[13:]) == [(CMD_STATUS, payload)]
    assert parser.feed(frame + frame) == [(CMD_STATUS, payload), (CMD_STATUS, payload)]
    assert parser.feed(b"\x00noise" + frame) == [(CMD_STATUS, payload)]


def test_frame_parser_recovers_after_bad_crc_and_illegal_length():
    payload = make_status_payload()
    good = build_frame(CMD_STATUS, payload)
    bad_crc = bytearray(good)
    bad_crc[-1] ^= 0xFF
    parser = FrameParser()

    assert parser.feed(bytes(bad_crc) + good) == [(CMD_STATUS, payload)]
    assert parser.feed(b"\xA5\x5A\x00" + good) == [(CMD_STATUS, payload)]
    assert parser.feed(b"\xA5\x5A\x42" + good) == [(CMD_STATUS, payload)]


def test_build_frame_accepts_99_byte_payload_and_rejects_larger_payloads():
    payload = bytes(range(MAX_PAYLOAD_SIZE))
    frame = build_frame(CMD_IMU_STATUS, payload)

    assert MAX_PAYLOAD_SIZE == 99
    assert frame[2] == 100
    assert frame[3] == CMD_IMU_STATUS
    assert frame[4:-1] == payload
    assert frame[-1] == independent_crc8(frame[2:-1])

    with pytest.raises(ValueError, match="payload too large"):
        build_frame(CMD_IMU_STATUS, bytes(MAX_PAYLOAD_SIZE + 1))


def test_decode_status_payload_checks_fixed_size_version_offsets_and_units():
    payload = make_status_payload(
        battery_mv=24123,
        speeds=(-100, 120, 340, -560),
        encoders=(100, -200, 300, -400),
        currents=(10, 111, 222, 333),
        targets=(-90, 130, 350, -570),
        outputs=(-1, 501, -502, 999),
    )

    status = decode_status_payload(payload)

    assert status.version == 2
    assert status.status_flags == 0x08
    assert status.control_source == 1
    assert status.motor_enabled_mask == 0b0110
    assert status.error_flags == 0x11223344
    assert status.latched_error_flags == 0x55667788
    assert math.isclose(status.battery_voltage, 24.123)
    assert status.motor_speed_mps == (-0.100, 0.120, 0.340, -0.560)
    assert status.encoder_count == (100, -200, 300, -400)
    assert status.motor_current_a == (0.010, 0.111, 0.222, 0.333)
    assert status.motor_target_mps == (-0.090, 0.130, 0.350, -0.570)
    assert status.motor_output_permille == (-1, 501, -502, 999)
    assert status.motor_speed_valid_mask == 0b0110
    assert status.encoder_anomaly_mask == 0xA5
    assert status.comm_health_flags == 0x5A


def test_decode_status_payload_rejects_non_65_byte_or_non_v2_payload():
    assert decode_status_payload(make_status_payload()[:-1]) is None
    assert decode_status_payload(make_status_payload(version=1)) is None


def test_decode_imu_status_payload_checks_size_version_offsets_and_units():
    payload = make_imu_status_payload()

    imu = decode_imu_status_payload(payload)

    assert imu.version == PROTOCOL_VERSION
    assert imu.accel_g == pytest.approx((0.1, -0.2, 0.3))
    assert imu.gyro_corrected_dps == pytest.approx((1.5, -2.5, 3.5))
    assert imu.euler_deg == pytest.approx((10.0, -20.0, 30.0))
    # wire [w=0.5, x=0.5, y=-0.5, z=0.5] → remapped to ROS [x, y, z, w]
    assert imu.quaternion == pytest.approx((0.5, -0.5, 0.5, 0.5))
    assert imu.timestamp_ms == 123456
    assert imu.sensor_time == 654321
    assert imu.sample_count == 42
    assert imu.quality_flags == 0x11223344
    assert imu.quality_counters == (1, 2, 3, 4, 5, 6, 7)
    assert imu.status_flags == 0x0B
    assert imu.temperature_c == -12


def test_decode_imu_status_payload_rejects_bad_size_or_version():
    assert decode_imu_status_payload(make_imu_status_payload()[:-1]) is None
    assert decode_imu_status_payload(make_imu_status_payload(version=1)) is None


def test_aggregate_status_uses_m2_left_and_m3_right_for_default_two_drive_layout():
    status = decode_status_payload(
        make_status_payload(
            enabled_mask=0b0110,
            speeds=(9, 123, 456, 9),
            encoders=(9, 1200, 3400, 9),
            currents=(9, 321, 654, 9),
            targets=(9, 111, 222, 9),
            speed_valid_mask=0b0110,
        )
    )

    agg = aggregate_status(status, wheel_track_width=0.176)

    assert agg.odom_trusted is True
    assert math.isclose(agg.left_speed_mps, 0.123)
    assert math.isclose(agg.right_speed_mps, 0.456)
    assert math.isclose(agg.left_current_a, 0.321)
    assert math.isclose(agg.right_current_a, 0.654)
    assert agg.left_encoder_count == 1200
    assert agg.right_encoder_count == 3400
    assert math.isclose(agg.vx_mps, (0.123 + 0.456) / 2.0)
    assert math.isclose(agg.wz_radps, (0.456 - 0.123) / 0.176)


def test_aggregate_status_averages_four_drive_sides_and_sums_side_currents():
    status = decode_status_payload(
        make_status_payload(
            enabled_mask=0b1111,
            speeds=(100, 300, 500, 700),
            encoders=(1000, 3000, 5000, 7000),
            currents=(100, 200, 300, 400),
            targets=(110, 310, 510, 710),
            speed_valid_mask=0b1111,
        )
    )

    agg = aggregate_status(status, wheel_track_width=0.176)

    assert agg.odom_trusted is True
    assert math.isclose(agg.left_speed_mps, 0.200)
    assert math.isclose(agg.right_speed_mps, 0.600)
    assert math.isclose(agg.left_current_a, 0.300)
    assert math.isclose(agg.right_current_a, 0.700)
    assert agg.left_encoder_count == 2000
    assert agg.right_encoder_count == 6000
    assert math.isclose(agg.left_target_mps, 0.210)
    assert math.isclose(agg.right_target_mps, 0.610)


def test_aggregate_status_marks_odom_untrusted_when_side_missing_or_speed_invalid():
    missing_side = decode_status_payload(make_status_payload(enabled_mask=0b0010, speed_valid_mask=0b0010))
    invalid_speed = decode_status_payload(make_status_payload(enabled_mask=0b0110, speed_valid_mask=0b0010))

    assert aggregate_status(missing_side, wheel_track_width=0.176).odom_trusted is False
    assert aggregate_status(missing_side, wheel_track_width=0.176).vx_mps == 0.0
    assert aggregate_status(invalid_speed, wheel_track_width=0.176).odom_trusted is False
    assert aggregate_status(invalid_speed, wheel_track_width=0.176).wz_radps == 0.0


def test_aggregate_status_drive_mode_differential_default_is_unchanged():
    status = decode_status_payload(
        make_status_payload(
            enabled_mask=0b0110,
            speeds=(9, 123, 456, 9),
            encoders=(9, 1200, 3400, 9),
            currents=(9, 321, 654, 9),
            targets=(9, 111, 222, 9),
            speed_valid_mask=0b0110,
        )
    )

    agg_default = aggregate_status(status, wheel_track_width=0.176)
    agg_explicit = aggregate_status(status, wheel_track_width=0.176, drive_mode="differential")

    assert agg_default == agg_explicit


def test_aggregate_status_rejects_unknown_drive_mode():
    status = decode_status_payload(make_status_payload(enabled_mask=0b0110, speed_valid_mask=0b0110))

    with pytest.raises(ValueError, match="Unknown drive_mode"):
        aggregate_status(status, wheel_track_width=0.176, drive_mode="four_wheel")


def test_command_stream_sends_enable_one_while_active_then_single_release_on_timeout():
    stream = CommandStream(cmd_timeout_sec=0.25, keepalive_sec=0.10)
    sent = []

    stream.update_command(0.2, 0.1, now_sec=1.0)
    stream.tick(1.0, lambda payload: sent.append(payload))
    stream.tick(1.05, lambda payload: sent.append(payload))
    stream.tick(1.11, lambda payload: sent.append(payload))
    stream.tick(1.30, lambda payload: sent.append(payload))
    stream.tick(1.80, lambda payload: sent.append(payload))

    decoded = [struct.unpack("<ffBB", payload) for payload in sent]
    assert len(decoded) == 3
    assert decoded[0][2:] == (1, 2)
    assert decoded[1][2:] == (1, 2)
    assert decoded[2] == (0.0, 0.0, 0, 2)
    assert math.isclose(decoded[0][0], 0.2, abs_tol=1e-6)
    assert math.isclose(decoded[0][1], 0.1, abs_tol=1e-6)
    assert math.isclose(decoded[1][0], 0.2, abs_tol=1e-6)
    assert math.isclose(decoded[1][1], 0.1, abs_tol=1e-6)


# ── Golden vector tests ────────────────────────────────────────────────
# Vectors from SlamRobot_Chassis_Control/tests/fixtures/upper_v2_golden.json,
# generated by the STM32 firmware's UpperProtocol_BuildFrame().

GOLDEN_FRAMES = {
    "status_v2": (
        0x81,
        65,
        bytes.fromhex(
            "a55a4281020f030f78563412efcdab903930e2040cfe010030f801000000feffffffffffff7f"
            "000000806400b0046009fffffa0006ffdc0524fa64009cffe80318fc050a156d"
        ),
    ),
    "diagnostic_v1": (
        0x82,
        28,
        bytes.fromhex(
            "a55a1d820201010b04030201141312112301000024232221343332314443424149"
        ),
    ),
    "imu_temp_23c": (
        0x83,
        99,
        bytes.fromhex(
            "a55a6483020000003e000080be0000803f0000c03f000020c00000604000002041"
            "0000a0c10000f0410000803f000000000000000000000000040302010807060514"
            "131211242322210100000002000000030000000400000005000000060000000700"
            "00000b170e"
        ),
    ),
    "imu_temp_minus_41c": (
        0x83,
        99,
        bytes.fromhex(
            "a55a6483020000003e000080be0000803f0000c03f000020c00000604000002041"
            "0000a0c10000f0410000803f000000000000000000000000040302010807060514"
            "131211242322210100000002000000030000000400000005000000060000000700"
            "00000bd7c4"
        ),
    ),
    "imu_temp_87c": (
        0x83,
        99,
        bytes.fromhex(
            "a55a6483020000003e000080be0000803f0000c03f000020c00000604000002041"
            "0000a0c10000f0410000803f000000000000000000000000040302010807060514"
            "131211242322210100000002000000030000000400000005000000060000000700"
            "00000b5748"
        ),
    ),
}


def _parse_golden_frame(hex_bytes: bytes):
    """Parse a complete golden frame (including SOF) into (cmd, payload, expected_crc)."""
    assert hex_bytes[:2] == b"\xA5\x5A"
    length = hex_bytes[2]
    cmd = hex_bytes[3]
    body = hex_bytes[2 : 3 + length]  # length + cmd + payload
    expected_crc = hex_bytes[3 + length]
    payload = hex_bytes[4 : 3 + length]
    return cmd, payload, body, expected_crc


def test_golden_vector_crc8_matches_stm32():
    """Verify CRC8 for all golden frames matches the STM32-generated CRC."""
    for name, (expected_cmd, expected_payload_len, frame) in GOLDEN_FRAMES.items():
        cmd, payload, body, expected_crc = _parse_golden_frame(frame)
        assert cmd == expected_cmd, f"{name}: cmd mismatch"
        assert len(payload) == expected_payload_len, f"{name}: payload len mismatch"
        assert crc8(body) == expected_crc, f"{name}: CRC8 mismatch"


def test_golden_vector_decode_status():
    """Decode the golden STATUS frame and verify key fields."""
    _, _, frame = GOLDEN_FRAMES["status_v2"]
    cmd, payload, body, expected_crc = _parse_golden_frame(frame)
    status = decode_status_payload(payload)
    assert status is not None
    assert status.version == 2
    assert status.status_flags == (STATUS_FLAG_ESTOP | STATUS_FLAG_FAULT_STOP | 0x04 | STATUS_FLAG_SPEED_VALID_ALL)
    assert status.control_source == 3
    assert status.motor_enabled_mask == 0x0F
    assert status.error_flags == 0x12345678
    assert status.latched_error_flags == 0x90ABCDEF
    assert status.battery_voltage == pytest.approx(12.345)
    assert status.motor_speed_mps == pytest.approx((1.250, -0.500, 0.001, -2.000))
    assert status.encoder_count == (1, -2, 2147483647, -2147483648)
    assert status.motor_current_a == pytest.approx((0.100, 1.200, 2.400, 65.535))
    assert status.motor_target_mps == pytest.approx((0.250, -0.250, 1.500, -1.500))
    assert status.motor_output_permille == (100, -100, 1000, -1000)


def test_golden_vector_decode_imu_status_quaternion_remap():
    """Decode the golden IMU_STATUS (23°C) and verify quaternion is remapped to [x,y,z,w]."""
    _, _, frame = GOLDEN_FRAMES["imu_temp_23c"]
    cmd, payload, body, expected_crc = _parse_golden_frame(frame)
    imu = decode_imu_status_payload(payload)
    assert imu is not None
    assert imu.version == 2
    assert imu.accel_g == pytest.approx((0.125, -0.25, 1.0))
    assert imu.gyro_corrected_dps == pytest.approx((1.5, -2.5, 3.5))
    assert imu.euler_deg == pytest.approx((10.0, -20.0, 30.0))
    # Wire: [w=1.0, x=0.0, y=0.0, z=0.0] → ROS: [x=0, y=0, z=0, w=1.0]
    assert imu.quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert imu.timestamp_ms == 0x01020304
    assert imu.sensor_time == 0x05060708
    assert imu.sample_count == 0x11121314
    assert imu.quality_flags == 0x21222324
    assert imu.quality_counters == (1, 2, 3, 4, 5, 6, 7)
    assert imu.status_flags == 0x0B
    assert imu.temperature_c == 23


def test_golden_vector_decode_imu_temperature_variants():
    """Verify IMU temperature decoding at -41°C and 87°C."""
    _, _, frame_cold = GOLDEN_FRAMES["imu_temp_minus_41c"]
    _, _, frame_hot = GOLDEN_FRAMES["imu_temp_87c"]
    imu_cold = decode_imu_status_payload(_parse_golden_frame(frame_cold)[1])
    imu_hot = decode_imu_status_payload(_parse_golden_frame(frame_hot)[1])
    assert imu_cold.temperature_c == -41
    assert imu_hot.temperature_c == 87


def test_golden_vector_decode_diagnostic():
    """Decode the golden DIAGNOSTIC frame and verify all fields."""
    _, _, frame = GOLDEN_FRAMES["diagnostic_v1"]
    cmd, payload, body, expected_crc = _parse_golden_frame(frame)
    diag = decode_diagnostic_payload(payload)
    assert diag is not None
    assert diag.protocol_version == 2
    assert diag.schema_version == 1
    assert diag.post_done == 1
    assert diag.imu_status_flags == 0x0B
    assert diag.post_error_flags == 0x01020304
    assert diag.adc_invalid_reason_flags == 0x11121314
    assert diag.task_timeout_mask == 0x0123
    assert diag.imu_quality_flags == 0x21222324
    assert diag.reset_reason_flags == 0x31323334
    assert diag.uptime_ms == 0x41424344


def test_diagnostic_payload_rejects_wrong_size_and_version():
    """DIAGNOSTIC decode should reject non-28B or non-v2 payloads."""
    _, _, frame = GOLDEN_FRAMES["diagnostic_v1"]
    _, valid_payload, _, _ = _parse_golden_frame(frame)
    assert decode_diagnostic_payload(valid_payload[:-1]) is None
    bad_version = bytearray(valid_payload)
    bad_version[0] = 1
    assert decode_diagnostic_payload(bytes(bad_version)) is None


def test_golden_frames_pass_frame_parser():
    """All golden frames should be parsed successfully by FrameParser."""
    parser = FrameParser()
    for name, (expected_cmd, expected_len, frame) in GOLDEN_FRAMES.items():
        results = parser.feed(frame)
        assert len(results) == 1, f"{name}: expected 1 frame, got {len(results)}"
        parsed_cmd, parsed_payload = results[0]
        assert parsed_cmd == expected_cmd, f"{name}: cmd mismatch"
        assert len(parsed_payload) == expected_len, f"{name}: payload len mismatch"


def test_encode_line_ctrl():
    """LINE_CTRL encode produces 1-byte payload."""
    assert encode_line_ctrl_payload(True) == b"\x01"
    assert encode_line_ctrl_payload(False) == b"\x00"


def test_encode_clear_fault():
    """CLEAR_FAULT encode produces empty payload."""
    assert encode_clear_fault_payload() == b""


def test_build_frame_line_ctrl():
    """Build LINE_CTRL frame with correct cmd, payload, and CRC."""
    frame = build_frame(CMD_LINE_CTRL, encode_line_ctrl_payload(True))
    assert frame[:2] == b"\xA5\x5A"
    assert frame[2] == 2  # 1 cmd + 1 payload
    assert frame[3] == CMD_LINE_CTRL
    assert frame[4] == 0x01  # enabled=True
    assert frame[5] == crc8(frame[2:5])


def test_build_frame_clear_fault():
    """Build CLEAR_FAULT frame with zero-length payload."""
    frame = build_frame(CMD_CLEAR_FAULT, encode_clear_fault_payload())
    assert frame[:2] == b"\xA5\x5A"
    assert frame[2] == 1  # 1 cmd + 0 payload
    assert frame[3] == CMD_CLEAR_FAULT
    assert frame[4] == crc8(frame[2:4])


def test_build_frame_estop():
    """ESTOP encode produces 1-byte payload."""
    payload = encode_estop_payload(True)
    assert payload == b"\x01"
    assert encode_estop_payload(False) == b"\x00"


def test_imu_quaternion_distinct_values_remap():
    """Verify quaternion remapping with distinct values on wire."""
    # Wire: w=0.8, x=0.1, y=0.2, z=0.3
    payload = make_imu_status_payload(quaternion=(0.8, 0.1, 0.2, 0.3))
    imu = decode_imu_status_payload(payload)
    # ROS: [x=0.1, y=0.2, z=0.3, w=0.8]
    assert imu.quaternion == pytest.approx((0.1, 0.2, 0.3, 0.8))
