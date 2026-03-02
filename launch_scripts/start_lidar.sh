#!/bin/bash
# ============================================
# YDLIDAR X2 雷达启动脚本
# ============================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_WS="/home/robot/ros2_ws"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}    YDLIDAR X2 雷达启动系统${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}未检测到ROS2环境，正在设置...${NC}"
    source /opt/ros/foxy/setup.bash
    source $ROS_WS/install/setup.bash
else
    source $ROS_WS/install/setup.bash
fi

# 检查雷达设备
echo -e "${YELLOW}检查雷达设备...${NC}"
if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}✓ 找到雷达设备: /dev/ttyUSB0${NC}"
else
    echo -e "${RED}✗ 未找到雷达设备 /dev/ttyUSB0${NC}"
    echo -e "${YELLOW}请检查：${NC}"
    echo "  1. 雷达是否已连接"
    echo "  2. USB线是否正常"
    echo "  3. 运行以下命令检查设备: ls -la /dev/ttyUSB*"
    exit 1
fi

# 启动雷达
echo ""
echo -e "${GREEN}正在启动传感器层（雷达）...${NC}"
echo -e "${YELLOW}配置文件: X2.yaml${NC}"
echo -e "${YELLOW}波特率: 115200${NC}"
echo -e "${YELLOW}测距范围: 0.1-12.0m${NC}"
echo ""
echo -e "${YELLOW}按 Ctrl+C 停止雷达${NC}"
echo ""

ros2 launch robot_bringup sensors.launch.py \
    use_lidar:=true \
    use_camera:=false \
    use_rviz:=true

echo ""
echo -e "${GREEN}雷达已停止${NC}"
