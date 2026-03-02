#!/bin/bash
# ============================================
# 导航模式启动脚本
# 功能：雷达 + 底盘 + 定位 + Nav2 + RViz
# ============================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ROS_WS="/home/robot/ros2_ws"
DEFAULT_MAP="/home/robot/ros2_maps/latest.yaml"
MAP_FILE="${1:-$DEFAULT_MAP}"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    导航模式启动${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

source /opt/ros/foxy/setup.bash
source $ROS_WS/install/setup.bash

if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}✗ 地图文件不存在: $MAP_FILE${NC}"
    echo -e "${YELLOW}用法: ./start_navigation.sh /abs/path/to/map.yaml${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 使用地图: $MAP_FILE${NC}"
echo -e "${YELLOW}按 Ctrl+C 停止${NC}"
echo ""

ros2 launch robot_bringup system.launch.py \
    mode:=navigation \
    use_camera:=true \
    use_base:=true \
    use_rviz:=true \
    map_file:=$MAP_FILE \
    base_port:=/dev/ttyUSB1 \
    base_baudrate:=115200
