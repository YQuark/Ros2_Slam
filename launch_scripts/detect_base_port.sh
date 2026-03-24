#!/bin/bash

set -euo pipefail

emit_first=1
if [ "${1:-}" = "--all" ]; then
    emit_first=0
fi

declare -A SEEN=()
RESULTS=()
MATCHED=()

add_port() {
    local candidate="$1"
    local resolved="$candidate"

    [ -e "$candidate" ] || return 0

    if resolved="$(readlink -f "$candidate" 2>/dev/null)"; then
        :
    else
        resolved="$candidate"
    fi

    [ -n "$resolved" ] || return 0
    [ -e "$resolved" ] || return 0

    if [ -n "${SEEN[$resolved]:-}" ]; then
        return 0
    fi

    SEEN[$resolved]=1
    RESULTS+=("$resolved")
}

for pattern in \
    /dev/serial/by-id/*CP210* \
    /dev/serial/by-id/*cp210* \
    /dev/serial/by-id/*Silicon_Labs* \
    /dev/serial/by-id/*USB*UART*
do
    for candidate in $pattern; do
        [ -e "$candidate" ] || continue
        add_port "$candidate"
    done
done

for port in /dev/ttyUSB*; do
    [ -e "$port" ] || continue

    if command -v udevadm >/dev/null 2>&1; then
        props="$(udevadm info -q property -n "$port" 2>/dev/null || true)"
        if printf '%s\n' "$props" | grep -Eiq 'ID_VENDOR_ID=10c4|ID_MODEL_ID=ea60|CP210'; then
            add_port "$port"
            continue
        fi
    fi

    tty_name="${port##*/}"
    driver_path="$(readlink -f "/sys/class/tty/${tty_name}/device/driver" 2>/dev/null || true)"
    if printf '%s\n' "$driver_path" | grep -q 'cp210x'; then
        add_port "$port"
    fi
done

if [ "${#RESULTS[@]}" -eq 0 ]; then
    exit 0
fi

probe_base_port() {
    local port="$1"

    python3 - "$port" <<'PY'
import sys
import time

try:
    import serial
except Exception:
    sys.exit(2)

port = sys.argv[1]

LINK_SOF = 0xA5
LINK_EOF = 0x5A
LINK_VER = 0x01
MSG_GET_STATUS = 0x02
FLAG_ACK_REQ = 0x01
FLAG_IS_ACK = 0x02


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    idx = 0
    while idx < len(data):
        code_pos = len(out)
        out.append(0)
        code = 1
        while idx < len(data) and data[idx] != 0 and code < 0xFF:
            out.append(data[idx])
            idx += 1
            code += 1
        out[code_pos] = code
        if idx < len(data) and data[idx] == 0:
            idx += 1
    return bytes(out)


def cobs_decode(data: bytes):
    out = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]
        if code == 0:
            return None
        idx += 1
        for _ in range(code - 1):
            if idx >= len(data):
                return None
            out.append(data[idx])
            idx += 1
        if code != 0xFF and idx < len(data):
            out.append(0)
    return bytes(out)


def build_get_status(seq: int) -> bytes:
    payload = b""
    header = bytes([LINK_SOF, LINK_VER, MSG_GET_STATUS, FLAG_ACK_REQ, seq]) + len(payload).to_bytes(2, "little")
    crc = crc16_ccitt_false(header[1:] + payload)
    frame = header + payload + crc.to_bytes(2, "little") + bytes([LINK_EOF])
    return cobs_encode(frame) + b"\x00"


def parse_frame(raw: bytes):
    dec = cobs_decode(raw)
    if dec is None or len(dec) < 10:
        return None
    if dec[0] != LINK_SOF or dec[-1] != LINK_EOF:
        return None
    if dec[1] != LINK_VER:
        return None
    payload_len = int.from_bytes(dec[5:7], "little")
    if len(dec) != payload_len + 10:
        return None
    payload = dec[7:7 + payload_len]
    recv_crc = int.from_bytes(dec[7 + payload_len:9 + payload_len], "little")
    calc_crc = crc16_ccitt_false(dec[1:7] + payload)
    if recv_crc != calc_crc:
        return None
    return dec[2], dec[3], dec[4], payload

try:
    ser = serial.Serial(port, 115200, timeout=0.2)
except Exception:
    sys.exit(1)

try:
    # Give the MCU a moment in case opening the UART triggers a reset.
    time.sleep(1.0)
    ser.reset_input_buffer()

    for seq in range(1, 4):
        ser.write(build_get_status(seq))
        ser.flush()

        deadline = time.time() + 0.8
        buf = bytearray()
        while time.time() < deadline:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)
                while True:
                    try:
                        end = buf.index(0)
                    except ValueError:
                        break
                    raw = bytes(buf[:end])
                    del buf[:end + 1]
                    if not raw:
                        continue
                    parsed = parse_frame(raw)
                    if not parsed:
                        continue
                    msg_type, flags, _seq, payload = parsed
                    if msg_type == MSG_GET_STATUS and (flags & FLAG_IS_ACK) and len(payload) >= 20:
                        print(port)
                        raise SystemExit(0)
            else:
                time.sleep(0.05)
except Exception:
    pass
finally:
    ser.close()
PY
}

for port in "${RESULTS[@]}"; do
    matched="$(probe_base_port "$port" 2>/dev/null || true)"
    if [ -n "$matched" ]; then
        MATCHED+=("$port")
    fi
done

if [ "${#MATCHED[@]}" -gt 0 ]; then
    RESULTS=("${MATCHED[@]}")
fi

if [ "$emit_first" -eq 1 ]; then
    printf '%s\n' "${RESULTS[0]}"
    exit 0
fi

printf '%s\n' "${RESULTS[@]}"
