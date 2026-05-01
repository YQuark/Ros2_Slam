#!/bin/bash
# ============================================
# 测试摄像头脚本
# ============================================

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_WS="/home/robot/ros2_ws"

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}    摄像头测试${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source $ROS_WS/install/setup.bash

# 检查摄像头设备
echo -e "${YELLOW}1. 检查摄像头设备...${NC}"
if [ -e /dev/video0 ]; then
    echo -e "${GREEN}✓ 找到 /dev/video0${NC}"
    VIDEO_DEVICE="/dev/video0"
elif [ -e /dev/video1 ]; then
    echo -e "${GREEN}✓ 找到 /dev/video1${NC}"
    VIDEO_DEVICE="/dev/video1"
else
    echo -e "${RED}✗ 未找到摄像头设备${NC}"
    echo -e "${YELLOW}请检查摄像头连接${NC}"
    exit 1
fi

# 检查astra_camera包
echo ""
echo -e "${YELLOW}2. 检查astra_camera包...${NC}"
if ros2 pkg list | grep -q "astra_camera"; then
    echo -e "${GREEN}✓ astra_camera包已安装${NC}"
else
    echo -e "${RED}✗ astra_camera包未安装${NC}"
    exit 1
fi

# 检查话题
echo ""
echo -e "${YELLOW}3. 启动摄像头（测试10秒）...${NC}"
echo -e "${YELLOW}正在启动，请稍候...${NC}"
echo ""

# 启动摄像头（后台运行）
ros2 launch ydlidar_ros2_driver astra_pro.launch.py &
CAMERA_PID=$!

# 等待启动
sleep 3

# 检查话题
echo -e "${YELLOW}4. 检查摄像头话题...${NC}"
TOPICS=$(ros2 topic list 2>/dev/null | grep -i camera)
if [ -n "$TOPICS" ]; then
    echo -e "${GREEN}✓ 摄像头话题已发布:${NC}"
    echo "$TOPICS" | while read topic; do
        echo -e "${GREEN}  • $topic${NC}"
    done

    # 显示话题信息
    echo ""
    echo -e "${YELLOW}5. 话题信息:${NC}"
    ros2 topic info /camera/color/image_raw 2>/dev/null || ros2 topic info /camera/depth/image_rect_raw 2>/dev/null

else
    echo -e "${RED}✗ 未找到摄像头话题${NC}"
    kill $CAMERA_PID 2>/dev/null
    exit 1
fi

echo ""
echo -e "${GREEN}✓ 摄像头测试通过！${NC}"
echo ""
echo -e "${YELLOW}测试完成，正在停止摄像头...${NC}"
kill $CAMERA_PID 2>/dev/null
wait $CAMERA_PID 2>/dev/null

echo ""
echo -e "${GREEN}可以正常运行摄像头了！${NC}"
echo -e "${YELLOW}使用以下命令启动:${NC}"
echo "  ./robot.sh sensor camera"
