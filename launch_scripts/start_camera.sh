#!/bin/bash
# ============================================
# Astra Pro 摄像头启动脚本
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

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    Astra Pro 摄像头启动${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 设置ROS2环境
source /opt/ros/foxy/setup.bash
source $ROS_WS/install/setup.bash

# 检查摄像头设备
echo -e "${YELLOW}检查摄像头设备...${NC}"
if [ -e /dev/video0 ]; then
    echo -e "${GREEN}✓ 找到摄像头设备: /dev/video0${NC}"
elif [ -e /dev/video1 ]; then
    echo -e "${GREEN}✓ 找到摄像头设备: /dev/video1${NC}"
else
    echo -e "${RED}✗ 未找到摄像头设备${NC}"
    echo -e "${YELLOW}请检查摄像头连接${NC}"
    exit 1
fi

# 启动摄像头
echo ""
echo -e "${GREEN}正在启动Astra Pro摄像头...${NC}"
echo -e "${YELLOW}功能: RGB图像 + 深度图像 + IR图像${NC}"
echo ""
echo -e "${YELLOW}话题:${NC}"
echo "  • /camera/color/image_raw - RGB图像"
echo "  • /camera/depth/image_rect_raw - 深度图像"
echo "  • /camera/infra/image_raw - IR图像"
echo ""
echo -e "${YELLOW}按 Ctrl+C 停止摄像头${NC}"
echo ""

ros2 launch robot_bringup sensors.launch.py \
    use_lidar:=false \
    use_camera:=true

echo ""
echo -e "${GREEN}摄像头已停止${NC}"
