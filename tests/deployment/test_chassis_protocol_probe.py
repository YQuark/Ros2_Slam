import importlib.util
from pathlib import Path
import struct

from stm32_robot_bridge.framing import build_frame
from stm32_robot_bridge.protocol_v3 import ACK_APPLIED, CMD_STATUS, STATUS_PAYLOAD_SIZE


ROOT = Path(__file__).resolve().parents[2]
PROBE_PATH = ROOT / "launch_scripts" / "chassis_protocol_probe.py"


def status_frame():
    payload = bytearray(STATUS_PAYLOAD_SIZE)
    payload[0:4] = bytes((3, 0, 2, 0x0F))
    struct.pack_into("<H", payload, 12, 12345)
    payload[62:65] = bytes((0x0F, 0, 0x15))
    struct.pack_into("<IIQII", payload, 65, 7, 350, 99, 5, 5)
    payload[91] = ACK_APPLIED
    return build_frame(CMD_STATUS, payload)


def load_probe_module():
    spec = importlib.util.spec_from_file_location("chassis_protocol_probe", PROBE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ChunkedSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)

    def read(self, _size):
        return self.chunks.pop(0) if self.chunks else b""


def test_probe_decodes_v3_status_across_serial_chunks():
    probe = load_probe_module()
    frame = status_frame()
    serial_port = ChunkedSerial(
        [
            b"garbage" + frame[:3],
            frame[3:31],
            frame[31:],
        ]
    )

    status = probe.find_status(serial_port, timeout_sec=0.2)

    assert status is not None
    assert status.version == 3
    assert status.control_source == 2
    assert status.battery_voltage == 12.345
    assert status.comm_health_flags == 0x15
    assert status.status_sequence == 7
    assert status.last_received_session_id == 99


def test_chassis_shell_diagnostics_share_the_protocol_probe():
    check_chassis = (ROOT / "launch_scripts" / "check_chassis.sh").read_text(encoding="utf-8")
    detect_base_port = (ROOT / "launch_scripts" / "detect_base_port.sh").read_text(encoding="utf-8")

    for script in (check_chassis, detect_base_port):
        assert "chassis_protocol_probe.py" in script
        assert "crc << 1" not in script
        assert "length > 65" not in script
        assert "len(payload) != 64" not in script
