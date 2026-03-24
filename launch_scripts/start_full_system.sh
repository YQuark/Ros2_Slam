#!/bin/bash
# ============================================
# 完整系统启动脚本
# 功能：雷达 + 摄像头 + 下位机 + SLAM + 导航
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
echo -e "${BLUE}    完整系统启动${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}系统组件:${NC}"
echo "  • YDLIDAR X2 激光雷达"
echo "  • Astra Pro 深度摄像头"
echo "  • 下位机控制（STM32串口桥接）"
echo "  • SLAM 建图/导航"
echo "  • RViz2 可视化"
echo ""

# 设置ROS2环境
echo -e "${YELLOW}正在设置ROS2环境...${NC}"
source /opt/ros/foxy/setup.bash
source $ROS_WS/install/setup.bash

# 检查所有设备
echo ""
echo -e "${YELLOW}检查系统设备...${NC}"

# 检查雷达
if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}✓ 雷达设备: /dev/ttyUSB0${NC}"
else
    echo -e "${RED}✗ 雷达设备未找到${NC}"
fi

# 检查摄像头
if [ -e /dev/video0 ]; then
    echo -e "${GREEN}✓ 摄像头设备: /dev/video0${NC}"
else
    echo -e "${YELLOW}⚠ 摄像头设备未找到（可选）${NC}"
fi

# 检查下位机
if [ "$BASE_PORT" = "auto" ]; then
    echo -e "${YELLOW}⚠ 下位机 CP2102 串口暂未检测到，桥接节点将以 auto 模式持续重试${NC}"
else
    echo -e "${GREEN}✓ 下位机设备: ${BASE_PORT}${NC}"
fi

echo ""
echo -e "${GREEN}正在启动完整系统（统一入口）...${NC}"
echo -e "${YELLOW}模式: mapping${NC}"
echo -e "${YELLOW}按 Ctrl+C 停止所有服务${NC}"
echo ""

ros2 launch robot_bringup system.launch.py \
    mode:=mapping \
    use_camera:=true \
    use_base:=true \
    use_rviz:=true \
    base_port:=${BASE_PORT} \
    base_baudrate:=115200
