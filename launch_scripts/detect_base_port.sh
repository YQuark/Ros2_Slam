#!/bin/bash

set -euo pipefail

emit_first=1
passive_probe=0

while [ $# -gt 0 ]; do
    case "$1" in
        --all)
            emit_first=0
            ;;
        --probe)
            passive_probe=1
            ;;
    esac
    shift
done

BASE_PORT_HINT="${ROBOT_BASE_PORT_HINT:-${BASE_PORT_HINT:-/dev/serial0}}"

resolve_port() {
    local candidate="$1"
    readlink -f "$candidate" 2>/dev/null || printf '%s\n' "$candidate"
}

add_if_exists() {
    local candidate="$1"
    [ -e "$candidate" ] || return 0
    resolve_port "$candidate"
}

probe_v2_status() {
    local port="$1"

    python3 - "$port" <<'PY'
import sys
import time

try:
    import serial
except Exception:
    sys.exit(2)

port = sys.argv[1]
FRAME0 = 0xA5
FRAME1 = 0x5A
CMD_STATUS = 0x81
STATUS_PAYLOAD_SIZE = 64
MAX_LENGTH = 65


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


def parse_frames(buffer: bytearray):
    frames = []
    while True:
        header = -1
        for index in range(max(0, len(buffer) - 1)):
            if buffer[index] == FRAME0 and buffer[index + 1] == FRAME1:
                header = index
                break
        if header < 0:
            if buffer and buffer[-1] == FRAME0:
                del buffer[:-1]
            else:
                buffer.clear()
            return frames
        if header > 0:
            del buffer[:header]
        if len(buffer) < 3:
            return frames

        length = buffer[2]
        if length == 0 or length > MAX_LENGTH:
            del buffer[0]
            continue
        frame_len = length + 4
        if len(buffer) < frame_len:
            return frames

        body = bytes(buffer[2:3 + length])
        if crc8(body) != buffer[frame_len - 1]:
            del buffer[0]
            continue
        cmd = buffer[3]
        payload = bytes(buffer[4:3 + length])
        frames.append((cmd, payload))
        del buffer[:frame_len]


try:
    ser = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.05, rtscts=False, dsrdtr=False)
except Exception:
    sys.exit(1)

try:
    for attr in ("dtr", "rts"):
        try:
            setattr(ser, attr, False)
        except Exception:
            pass
    deadline = time.monotonic() + 1.2
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
            for cmd, payload in parse_frames(buf):
                if cmd == CMD_STATUS and len(payload) == STATUS_PAYLOAD_SIZE and payload[0] == 2:
                    print(port)
                    raise SystemExit(0)
        else:
            time.sleep(0.02)
finally:
    ser.close()

sys.exit(1)
PY
}

RESULTS=()
if [ -n "$BASE_PORT_HINT" ]; then
    while IFS= read -r port; do
        [ -n "$port" ] && RESULTS+=("$port")
    done < <(add_if_exists "$BASE_PORT_HINT")
fi

if [ "$BASE_PORT_HINT" != "/dev/serial0" ]; then
    while IFS= read -r port; do
        [ -n "$port" ] && RESULTS+=("$port")
    done < <(add_if_exists "/dev/serial0")
fi

if [ -e "/dev/ttyAMA0" ]; then
    resolved_ama="$(resolve_port "/dev/ttyAMA0")"
    duplicate=0
    for port in "${RESULTS[@]:-}"; do
        if [ "$port" = "$resolved_ama" ]; then
            duplicate=1
            break
        fi
    done
    if [ "$duplicate" -eq 0 ]; then
        RESULTS+=("$resolved_ama")
    fi
fi

if [ "${#RESULTS[@]}" -eq 0 ]; then
    exit 0
fi

if [ "$passive_probe" -eq 1 ]; then
    MATCHED=()
    for port in "${RESULTS[@]}"; do
        matched="$(probe_v2_status "$port" 2>/dev/null || true)"
        if [ -n "$matched" ]; then
            MATCHED+=("$matched")
        fi
    done
    RESULTS=("${MATCHED[@]}")
    if [ "${#RESULTS[@]}" -eq 0 ]; then
        exit 0
    fi
fi

if [ "$emit_first" -eq 1 ]; then
    printf '%s\n' "${RESULTS[0]}"
    exit 0
fi

printf '%s\n' "${RESULTS[@]}"
