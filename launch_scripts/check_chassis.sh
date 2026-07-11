#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/home/robot/ros2_ws/launch_scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

DETECT_BASE_PORT_SCRIPT="${SCRIPT_DIR}/detect_base_port.sh"
PROTOCOL_PROBE="${SCRIPT_DIR}/chassis_protocol_probe.py"
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
    python3 "$PROTOCOL_PROBE" summary "$port" || true
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
