import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PROBE_PATH = ROOT / "launch_scripts" / "chassis_protocol_probe.py"
STATUS_GOLDEN_FRAME = bytes.fromhex(
    "a55a4281020f030f78563412efcdab903930e2040cfe010030f801000000feffffffffffff7f"
    "000000806400b0046009fffffa0006ffdc0524fa64009cffe80318fc050a156d"
)


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


def test_probe_decodes_firmware_golden_status_across_serial_chunks():
    probe = load_probe_module()
    serial_port = ChunkedSerial(
        [
            b"garbage" + STATUS_GOLDEN_FRAME[:3],
            STATUS_GOLDEN_FRAME[3:31],
            STATUS_GOLDEN_FRAME[31:],
        ]
    )

    status = probe.find_status(serial_port, timeout_sec=0.2)

    assert status is not None
    assert status.version == 2
    assert status.control_source == 3
    assert status.battery_voltage == 12.345
    assert status.comm_health_flags == 0x15


def test_chassis_shell_diagnostics_share_the_protocol_probe():
    check_chassis = (ROOT / "launch_scripts" / "check_chassis.sh").read_text(encoding="utf-8")
    detect_base_port = (ROOT / "launch_scripts" / "detect_base_port.sh").read_text(encoding="utf-8")

    for script in (check_chassis, detect_base_port):
        assert "chassis_protocol_probe.py" in script
        assert "crc << 1" not in script
        assert "length > 65" not in script
        assert "len(payload) != 64" not in script
