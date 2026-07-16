#!/usr/bin/env python3
"""Passive upper-v3 STATUS probe shared by chassis shell diagnostics."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Optional, Tuple


try:
    from stm32_robot_bridge.framing import FrameParser
    from stm32_robot_bridge.protocol_v3 import CMD_STATUS, StatusPayload, decode_status_payload
except ImportError:
    workspace_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(workspace_root / "src" / "stm32_robot_bridge"))
    from stm32_robot_bridge.framing import FrameParser  # type: ignore[no-redef]
    from stm32_robot_bridge.protocol_v3 import (  # type: ignore[no-redef]
        CMD_STATUS,
        StatusPayload,
        decode_status_payload,
    )


def find_status(serial_port, timeout_sec: float = 1.2) -> Optional[StatusPayload]:
    parser = FrameParser()
    deadline = time.monotonic() + max(float(timeout_sec), 0.0)

    while time.monotonic() < deadline:
        chunk = serial_port.read(256)
        if not chunk:
            time.sleep(0.02)
            continue
        for command, payload in parser.feed(chunk):
            if command != CMD_STATUS:
                continue
            status = decode_status_payload(payload)
            if status is not None:
                return status
    return None


def open_port(port: str):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("python3-serial not installed") from exc

    serial_port = serial.Serial(
        port,
        115200,
        timeout=0.05,
        write_timeout=0.05,
        rtscts=False,
        dsrdtr=False,
    )
    for attr in ("dtr", "rts"):
        try:
            setattr(serial_port, attr, False)
        except Exception:
            pass
    return serial_port


def read_status(port: str, timeout_sec: float) -> Tuple[Optional[StatusPayload], Optional[str]]:
    try:
        serial_port = open_port(port)
    except Exception as exc:
        return None, str(exc)

    try:
        return find_status(serial_port, timeout_sec), None
    finally:
        serial_port.close()


def print_summary(status: StatusPayload) -> None:
    print(
        f"  protocol={status.version} control_source={status.control_source} "
        f"status_flags=0x{status.status_flags:02X}"
    )
    print(
        f"  enabled_mask=0x{status.motor_enabled_mask:02X} "
        f"speed_valid_mask=0x{status.motor_speed_valid_mask:02X} "
        f"battery={status.battery_voltage:.2f}V"
    )
    print(
        f"  error_flags=0x{status.error_flags:08X} "
        f"latched_error_flags=0x{status.latched_error_flags:08X}"
    )
    print(
        f"  status_sequence={status.status_sequence} "
        f"sample_timestamp_ms={status.sample_timestamp_ms} "
        f"ack_session={status.last_received_session_id} "
        f"ack_sequence={status.last_applied_sequence}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe passive STM32 upper-v3 STATUS frames")
    parser.add_argument("mode", choices=("probe", "summary"))
    parser.add_argument("port")
    parser.add_argument("--timeout", type=float, default=1.2)
    args = parser.parse_args()

    status, error = read_status(args.port, args.timeout)
    if error is not None:
        print(error, file=sys.stderr)
        return 2
    if status is None:
        return 1

    if args.mode == "probe":
        print(args.port)
    else:
        print_summary(status)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
