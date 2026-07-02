import math
import struct

from stm32_robot_bridge.protocol_v2 import (
    CMD_SET_VELOCITY,
    CMD_STATUS,
    PROTOCOL_VERSION,
    STATUS_PAYLOAD_SIZE,
    CommandStream,
    FrameParser,
    aggregate_status,
    build_frame,
    decode_status_payload,
    encode_velocity_payload,
)


def independent_crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x5E) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
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
    payload[63] = 0xAA
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


def test_decode_status_payload_rejects_non_64_byte_or_non_v2_payload():
    assert decode_status_payload(make_status_payload()[:-1]) is None
    assert decode_status_payload(make_status_payload(version=1)) is None


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

    agg = aggregate_status(status, wheel_track_width=0.178)

    assert agg.odom_trusted is True
    assert math.isclose(agg.left_speed_mps, 0.123)
    assert math.isclose(agg.right_speed_mps, 0.456)
    assert math.isclose(agg.left_current_a, 0.321)
    assert math.isclose(agg.right_current_a, 0.654)
    assert agg.left_encoder_count == 1200
    assert agg.right_encoder_count == 3400
    assert math.isclose(agg.vx_mps, (0.123 + 0.456) / 2.0)
    assert math.isclose(agg.wz_radps, (0.456 - 0.123) / 0.178)


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

    agg = aggregate_status(status, wheel_track_width=0.178)

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

    assert aggregate_status(missing_side, wheel_track_width=0.178).odom_trusted is False
    assert aggregate_status(missing_side, wheel_track_width=0.178).vx_mps == 0.0
    assert aggregate_status(invalid_speed, wheel_track_width=0.178).odom_trusted is False
    assert aggregate_status(invalid_speed, wheel_track_width=0.178).wz_radps == 0.0


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
