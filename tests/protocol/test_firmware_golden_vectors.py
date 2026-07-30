import hashlib
import json
import os
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "stm32_robot_bridge"))

from stm32_robot_bridge.framing import FrameParser
from stm32_robot_bridge.protocol_v3 import (
    CMD_DIAGNOSTIC,
    CMD_HELLO,
    CMD_IMU_STATUS,
    CMD_SET_VELOCITY,
    CMD_STATUS,
    DIAGNOSTIC_PAYLOAD_SIZE,
    HELLO_PAYLOAD_SIZE,
    IMU_STATUS_PAYLOAD_SIZE,
    STATUS_PAYLOAD_SIZE,
    VELOCITY_PAYLOAD_SIZE,
    decode_diagnostic_payload,
    decode_hello_payload,
    decode_imu_status_payload,
    decode_status_payload,
)


FIXTURE = ROOT / "tests" / "protocol" / "fixtures" / "upper_v3_golden.json"


def load_document():
    document = json.loads(FIXTURE.read_text(encoding="utf-8"))
    assert document["schema"] == 1
    assert document["protocol_version"] == 3
    return document


def test_checked_out_firmware_fixture_matches_snapshot_when_available():
    firmware_root = os.environ.get("FIRMWARE_CONTRACT_DIR")
    if not firmware_root:
        return
    firmware_path = Path(firmware_root)
    assert firmware_path.is_dir(), f"firmware contract checkout is missing: {firmware_path}"
    upstream = firmware_path / "tests" / "fixtures" / "upper_v3_golden.json"
    assert upstream.is_file(), "pinned firmware must publish the Upper-v3 golden fixture"
    assert json.loads(upstream.read_text(encoding="utf-8")) == load_document()


def test_checked_out_firmware_schema_matches_host_codec_when_available():
    firmware_root = os.environ.get("FIRMWARE_CONTRACT_DIR")
    if not firmware_root:
        return
    schema_path = Path(firmware_root) / "docs" / "protocol" / "upper-v3.schema.json"
    assert schema_path.is_file(), "pinned firmware must publish the machine wire schema"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))

    assert schema["protocol_version"] == 3
    assert schema["byte_order"] == "little-endian"
    assert schema["session"]["new_session_rule"].startswith("only enable=0")
    expected_sizes = {
        "SET_VELOCITY": VELOCITY_PAYLOAD_SIZE,
        "HELLO": HELLO_PAYLOAD_SIZE,
        "STATUS": STATUS_PAYLOAD_SIZE,
        "DIAGNOSTIC": DIAGNOSTIC_PAYLOAD_SIZE,
        "IMU_STATUS": IMU_STATUS_PAYLOAD_SIZE,
    }
    for message, size in expected_sizes.items():
        assert schema["messages"][message]["payload_length"] == size
    status_fields = {field["name"]: field for field in schema["messages"]["STATUS"]["fields"]}
    assert status_fields["motor_speed_valid_mask"]["offset"] == 62
    assert status_fields["encoder_anomaly_mask"]["offset"] == 63
    assert status_fields["ack_flags"]["offset"] == 91


def test_v3_fixture_metadata_hash_and_every_frame_decode():
    document = load_document()
    assert hashlib.sha256(FIXTURE.read_bytes()).hexdigest() == (
        "8406a0f8d0b450d1fc1af27ef6b2b5c17742285c732eeb8705da4bd2855e2a0e"
    )
    frames = document["frames"]
    assert {item["name"] for item in frames} == {
        "hello_v3",
        "status_v3",
        "status_default_2wd",
        "status_4wd",
        "status_single_wheel_anomaly",
        "diagnostic_v1",
        "imu_temp_23c",
        "imu_temp_minus_41c",
        "imu_temp_87c",
    }
    decoders = {
        CMD_HELLO: decode_hello_payload,
        CMD_STATUS: decode_status_payload,
        CMD_IMU_STATUS: decode_imu_status_payload,
        CMD_DIAGNOSTIC: decode_diagnostic_payload,
    }
    for item in frames:
        frame = bytes.fromhex(item["frame_hex"])
        assert len(frame) == item["frame_length"]
        parsed = FrameParser().feed(frame)
        assert parsed == [(item["command"], frame[4:-1])]
        assert len(parsed[0][1]) == item["payload_length"]
        if item["command"] in decoders:
            assert decoders[item["command"]](parsed[0][1]) is not None


def test_v3_frames_parse_bytewise_and_with_deterministic_random_chunks():
    stream = b"".join(bytes.fromhex(item["frame_hex"]) for item in load_document()["frames"])
    bytewise = FrameParser()
    expected = []
    for value in stream:
        expected.extend(bytewise.feed(bytes((value,))))

    randomizer = random.Random(20260722)
    chunked = FrameParser()
    actual = []
    offset = 0
    while offset < len(stream):
        size = randomizer.randint(1, 23)
        actual.extend(chunked.feed(stream[offset : offset + size]))
        offset += size
    assert actual == expected and len(actual) == 9


def test_v3_parser_recovers_after_garbage_bad_crc_and_bad_length():
    frame = bytes.fromhex(load_document()["frames"][1]["frame_hex"])
    bad_crc = frame[:-1] + bytes((frame[-1] ^ 0xFF,))
    parser = FrameParser()
    parsed = parser.feed(b"garbage" + b"\xa5\x5a\x00" + bad_crc + frame)
    assert parsed == [(CMD_STATUS, frame[4:-1])]
    assert parser.stats.crc_errors >= 1
    assert parser.stats.bad_length >= 1
    assert parser.stats.resync_bytes >= len(b"garbage")
