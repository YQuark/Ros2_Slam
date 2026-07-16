import json
import os
from pathlib import Path
import random
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "stm32_robot_bridge"))

from stm32_robot_bridge.protocol_v2 import FrameParser


FIXTURE = ROOT / "tests" / "protocol" / "fixtures" / "upper_v2_golden.json"


def load_frames():
    document = json.loads(FIXTURE.read_text(encoding="utf-8"))
    assert document["schema"] == 1
    assert document["protocol_version"] == 2
    return document["frames"]


def test_checked_out_firmware_fixture_matches_snapshot_when_available():
    firmware_root = os.environ.get("FIRMWARE_CONTRACT_DIR")
    if not firmware_root:
        return
    upstream = Path(firmware_root) / "tests" / "fixtures" / "upper_v2_golden.json"
    assert json.loads(upstream.read_text(encoding="utf-8")) == json.loads(
        FIXTURE.read_text(encoding="utf-8")
    )


def test_firmware_fixture_metadata_and_every_frame_parse():
    frames = load_frames()
    assert {item["name"] for item in frames} == {
        "status_v2",
        "diagnostic_v1",
        "imu_temp_23c",
        "imu_temp_minus_41c",
        "imu_temp_87c",
    }

    for item in frames:
        frame = bytes.fromhex(item["frame_hex"])
        assert len(frame) == item["frame_length"]
        parsed = FrameParser().feed(frame)
        assert parsed == [(item["command"], frame[4:-1])]
        assert len(parsed[0][1]) == item["payload_length"]


def test_firmware_frames_parse_with_bytewise_and_random_chunking():
    stream = b"".join(bytes.fromhex(item["frame_hex"]) for item in load_frames())

    bytewise = FrameParser()
    bytewise_frames = []
    for value in stream:
        bytewise_frames.extend(bytewise.feed(bytes((value,))))
    assert len(bytewise_frames) == 5

    randomizer = random.Random(20260715)
    chunked = FrameParser()
    chunked_frames = []
    offset = 0
    while offset < len(stream):
        size = randomizer.randint(1, 23)
        chunked_frames.extend(chunked.feed(stream[offset : offset + size]))
        offset += size
    assert chunked_frames == bytewise_frames


def test_parser_recovers_after_garbage_bad_crc_and_bad_length():
    frame = bytes.fromhex(load_frames()[0]["frame_hex"])
    bad_crc = frame[:-1] + bytes((frame[-1] ^ 0xFF,))
    parser = FrameParser()

    parsed = parser.feed(b"garbage" + b"\xA5\x5A\x00" + bad_crc + frame)

    assert parsed == [(load_frames()[0]["command"], frame[4:-1])]
    assert parser.stats.crc_errors >= 1
    assert parser.stats.bad_length >= 1
    assert parser.stats.resync_bytes >= len(b"garbage")
