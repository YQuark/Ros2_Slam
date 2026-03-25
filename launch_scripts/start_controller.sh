#!/bin/bash
# ============================================
# 下位机控制启动脚本
# ============================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_WS="/home/robot/ros2_ws"
BASE_PORT="${BASE_PORT:-auto}"

if [ "$BASE_PORT" = "auto" ] && [ -x "$SCRIPT_DIR/detect_base_port.sh" ]; then
    DETECTED_BASE_PORT="$($SCRIPT_DIR/detect_base_port.sh)"
    if [ -n "$DETECTED_BASE_PORT" ]; then
        BASE_PORT="$DETECTED_BASE_PORT"
    fi
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    下位机控制启动${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 设置ROS2环境
source /opt/ros/foxy/setup.bash
source $ROS_WS/install/setup.bash

# 检查下位机连接
echo -e "${YELLOW}检查下位机连接...${NC}"
if [ "$BASE_PORT" = "auto" ]; then
    echo -e "${YELLOW}⚠ 暂未检测到 CP2102 下位机串口，桥接节点将以 auto 模式持续重试${NC}"
else
    echo -e "${GREEN}✓ 下位机设备: ${BASE_PORT}${NC}"
fi

echo ""
echo -e "${GREEN}启动 STM32 串口桥接节点...${NC}"
echo -e "${YELLOW}按 Ctrl+C 停止${NC}"
echo ""

ros2 launch stm32_robot_bridge stm32_bridge.launch.py \
    port:=${BASE_PORT} \
    baudrate:=115200
