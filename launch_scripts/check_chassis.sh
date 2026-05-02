#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/home/robot/ros2_ws/launch_scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

DETECT_BASE_PORT_SCRIPT="${SCRIPT_DIR}/detect_base_port.sh"

print_header "底盘诊断"

# 1. 检测 CP2102 串口
echo -e "${BLUE}1. 串口检测${NC}"
if [ -x "$DETECT_BASE_PORT_SCRIPT" ]; then
    DETECTED=$("$DETECT_BASE_PORT_SCRIPT" --probe 2>/dev/null || true)
    if [ -n "$DETECTED" ]; then
        echo -e "${GREEN}✓ 检测到底盘串口: ${DETECTED}${NC}"
    else
        echo -e "${YELLOW}⚠ 未探测到底盘串口${NC}"
    fi
else
    echo -e "${YELLOW}⚠ detect_base_port.sh 不可用${NC}"
fi

# 列出所有 ttyUSB
echo "  可用串口设备:"
for port in /dev/ttyUSB* /dev/serial/by-id/*; do
    if [ -e "$port" ]; then
        echo "    $port"
    fi
done 2>/dev/null || echo "    (无)"
echo ""

# 2. 检查串口权限
echo -e "${BLUE}2. 串口权限${NC}"
for port in /dev/ttyUSB*; do
    if [ -e "$port" ]; then
        if [ -r "$port" ] && [ -w "$port" ]; then
            echo -e "${GREEN}✓ ${port}: 可读写${NC}"
        else
            echo -e "${RED}✗ ${port}: 权限不足${NC}"
            echo "    修复: sudo usermod -aG dialout \$USER && 重新登录"
        fi
    fi
done 2>/dev/null || echo -e "${YELLOW}⚠ 无 /dev/ttyUSB 设备${NC}"
echo ""

# 3. 检查 stm32_bridge 节点
echo -e "${BLUE}3. stm32_bridge 节点${NC}"
if ros2_node_exists "/stm32_bridge"; then
    echo -e "${GREEN}✓ /stm32_bridge 节点正在运行${NC}"
    echo "  检查 /odom 话题..."
    if ros2 topic list 2>/dev/null | grep -q "/odom"; then
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
