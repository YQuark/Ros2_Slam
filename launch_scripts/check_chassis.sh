#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/home/robot/ros2_ws/launch_scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

DETECT_BASE_PORT_SCRIPT="${SCRIPT_DIR}/detect_base_port.sh"
BASE_PORT="${ROBOT_BASE_PORT_HINT:-${BASE_PORT:-/dev/serial0}}"

print_header "底盘诊断"

echo -e "${BLUE}1. GPIO UART 设备${NC}"
for port in "$BASE_PORT" /dev/serial0 /dev/ttyAMA0; do
    [ -n "$port" ] || continue
    if [ -e "$port" ]; then
        resolved="$(readlink -f "$port" 2>/dev/null || printf '%s' "$port")"
        echo -e "${GREEN}✓ ${port} -> ${resolved}${NC}"
        if [ -r "$port" ] && [ -w "$port" ]; then
            echo -e "${GREEN}  可读写${NC}"
        else
            echo -e "${RED}  权限不足: 需要当前用户可读写串口${NC}"
        fi
    else
        echo -e "${YELLOW}⚠ ${port}: 不存在${NC}"
    fi
done
echo ""

echo -e "${BLUE}2. 用户组${NC}"
if id -nG 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then
    echo -e "${GREEN}✓ 当前用户属于 dialout${NC}"
else
    echo -e "${YELLOW}⚠ 当前用户不在 dialout 组${NC}"
    echo "  修复: sudo usermod -aG dialout \$USER && 重新登录"
fi
echo ""

echo -e "${BLUE}3. v2 STATUS 被动探测${NC}"
if [ -x "$DETECT_BASE_PORT_SCRIPT" ]; then
    DETECTED="$("$DETECT_BASE_PORT_SCRIPT" --probe 2>/dev/null || true)"
    if [ -n "$DETECTED" ]; then
        echo -e "${GREEN}✓ 收到 v2 STATUS: ${DETECTED}${NC}"
    else
        echo -e "${YELLOW}⚠ 1 秒内未收到有效 v2 STATUS${NC}"
        echo "  检查: STM32 上电、USART3 接线、115200 8N1、树莓派 GPIO UART 是否启用"
    fi
else
    echo -e "${YELLOW}⚠ detect_base_port.sh 不可用${NC}"
fi

print_status_summary() {
    local port="$1"
    python3 - "$port" <<'PY'
import struct
import sys
import time

try:
    import serial
except Exception:
    sys.exit(0)

port = sys.argv[1]


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x5E) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


buf = bytearray()
try:
    ser = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.05, rtscts=False, dsrdtr=False)
except Exception:
    sys.exit(0)

try:
    deadline = time.monotonic() + 1.2
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(0.02)
        while True:
            header = -1
            for i in range(max(0, len(buf) - 1)):
                if buf[i] == 0xA5 and buf[i + 1] == 0x5A:
                    header = i
                    break
            if header < 0:
                if buf and buf[-1] == 0xA5:
                    del buf[:-1]
                else:
                    buf.clear()
                break
            if header > 0:
                del buf[:header]
            if len(buf) < 3:
                break
            length = buf[2]
            if length == 0 or length > 65:
                del buf[0]
                continue
            frame_len = length + 4
            if len(buf) < frame_len:
                break
            body = bytes(buf[2:3 + length])
            if crc8(body) != buf[frame_len - 1]:
                del buf[0]
                continue
            cmd = buf[3]
            payload = bytes(buf[4:3 + length])
            del buf[:frame_len]
            if cmd != 0x81 or len(payload) != 64 or payload[0] != 2:
                continue
            version = payload[0]
            status_flags = payload[1]
            control_source = payload[2]
            enabled_mask = payload[3]
            error_flags = struct.unpack_from("<I", payload, 4)[0]
            latched = struct.unpack_from("<I", payload, 8)[0]
            battery = struct.unpack_from("<H", payload, 12)[0] / 1000.0
            speed_valid = payload[62]
            print(f"  protocol={version} control_source={control_source} status_flags=0x{status_flags:02X}")
            print(f"  enabled_mask=0x{enabled_mask:02X} speed_valid_mask=0x{speed_valid:02X} battery={battery:.2f}V")
            print(f"  error_flags=0x{error_flags:08X} latched_error_flags=0x{latched:08X}")
            raise SystemExit(0)
finally:
    ser.close()
PY
}

if [ -n "${DETECTED:-}" ]; then
    print_status_summary "$DETECTED"
fi
echo ""

echo -e "${BLUE}4. stm32_bridge 节点${NC}"
if command -v ros2 >/dev/null 2>&1 && ros2_node_exists "/stm32_bridge"; then
    echo -e "${GREEN}✓ /stm32_bridge 节点正在运行${NC}"
    if ros2 topic list 2>/dev/null | grep -q "^/odom$"; then
        echo -e "${GREEN}✓ /odom 话题存在${NC}"
    else
        echo -e "${YELLOW}⚠ /odom 话题未发布${NC}"
    fi
else
    echo -e "${YELLOW}⚠ /stm32_bridge 节点未运行${NC}"
    echo "  启动: ./robot.sh base"
fi
echo ""

echo -e "${GREEN}底盘诊断完成${NC}"
