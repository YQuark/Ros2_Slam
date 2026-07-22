import hashlib
import json
import os
from pathlib import Path
import random
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "stm32_robot_bridge"))

from stm32_robot_bridge.framing import FrameParser
from stm32_robot_bridge.protocol_v3 import (
    CMD_DIAGNOSTIC,
    CMD_HELLO,
    CMD_IMU_STATUS,
    CMD_SET_VELOCITY,
    CMD_STATUS,
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
    if not upstream.is_file():
        pytest.skip("pinned firmware does not publish an Upper-v3 golden-vector fixture yet")
    assert json.loads(upstream.read_text(encoding="utf-8")) == load_document()


def test_v3_fixture_metadata_hash_and_every_frame_decode():
    document = load_document()
    assert document["source"] == "host-contract-unverified-by-firmware"
    assert hashlib.sha256(FIXTURE.read_bytes()).hexdigest() == (
        "fe51e983a2e219a2b279d3f95fe137b1038d515db43758cb13dad9d4c56fa8ae"
    )
    frames = document["frames"]
    assert {item["name"] for item in frames} == {
        "hello_v3",
        "status_v3",
        "imu_v3",
        "diagnostic_v3",
        "velocity_v3",
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
    assert actual == expected and len(actual) == 5


def test_v3_parser_recovers_after_garbage_bad_crc_and_bad_length():
    frame = bytes.fromhex(load_document()["frames"][1]["frame_hex"])
    bad_crc = frame[:-1] + bytes((frame[-1] ^ 0xFF,))
    parser = FrameParser()
    parsed = parser.feed(b"garbage" + b"\xa5\x5a\x00" + bad_crc + frame)
    assert parsed == [(CMD_STATUS, frame[4:-1])]
    assert parser.stats.crc_errors >= 1
    assert parser.stats.bad_length >= 1
    assert parser.stats.resync_bytes >= len(b"garbage")
