#!/bin/bash
# ============================================
# 导航模式启动脚本
# 功能：雷达 + (真实底盘/虚拟底盘) + 定位 + Nav2 + RViz
# 用法：./start_navigation.sh [map.yaml] [--fake-base] [--no-rviz]
# ============================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ROS_WS="/home/robot/ros2_ws"
DEFAULT_MAP="/home/robot/ros2_maps/latest.yaml"
MAP_FILE="$DEFAULT_MAP"
BASE_MODE="real"
USE_RVIZ=true

usage() {
    echo "用法: $0 [map.yaml] [--fake-base] [--no-rviz]"
    echo "  map.yaml      地图文件路径（默认: $DEFAULT_MAP）"
    echo "  --fake-base   使用虚拟底盘（发布 /odom 与 odom->base_link）"
    echo "  --no-rviz     不启动 RViz"
}

for arg in "$@"; do
    case "$arg" in
        --fake-base)
            BASE_MODE="fake"
            ;;
        --real-base)
            BASE_MODE="real"
            ;;
        --no-rviz)
            USE_RVIZ=false
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            if [[ "$arg" == --* ]]; then
                echo -e "${RED}参数错误: $arg${NC}"
                usage
                exit 1
            fi
            MAP_FILE="$arg"
            ;;
    esac
done

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    导航模式启动${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

source /opt/ros/foxy/setup.bash
source $ROS_WS/install/setup.bash

if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}✗ 地图文件不存在: $MAP_FILE${NC}"
    usage
    exit 1
fi

if [ "$BASE_MODE" = "real" ]; then
    if [ ! -e /dev/ttyUSB1 ]; then
        echo -e "${RED}✗ 未找到底盘串口: /dev/ttyUSB1${NC}"
        echo -e "${YELLOW}可改用虚拟底盘: $0 $MAP_FILE --fake-base${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}✓ 使用地图: $MAP_FILE${NC}"
echo -e "${YELLOW}底盘模式: ${BASE_MODE}${NC}"
if [ "$USE_RVIZ" = true ]; then
    echo -e "${YELLOW}RViz: 开启${NC}"
else
    echo -e "${YELLOW}RViz: 关闭${NC}"
fi
echo -e "${YELLOW}按 Ctrl+C 停止${NC}"
echo ""

ros2 launch robot_bringup system.launch.py \
    mode:=navigation \
    use_camera:=false \
    use_lidar:=true \
    base_mode:=${BASE_MODE} \
    use_base:=true \
    use_rviz:=${USE_RVIZ} \
    map_file:=$MAP_FILE \
    base_port:=/dev/ttyUSB1 \
    base_baudrate:=115200
