#!/bin/bash
# ============================================
# 键盘控制脚本
# 用于控制虚拟机器人移动进行建图
# ============================================

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_WS="/home/robot/ros2_ws"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    键盘控制模式${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 设置ROS2环境
source /opt/ros/foxy/setup.bash
source $ROS_WS/install/setup.bash

echo -e "${GREEN}正在启动键盘控制...${NC}"
echo ""
echo -e "${YELLOW}操作说明:${NC}"
echo "  i : 前进"
echo "  o : 后退"
echo "  j : 左转"
echo "  l : 右转"
echo "  k : 停止"
echo "  q : 退出"
echo ""
echo -e "${YELLOW}按任意键继续...${NC}"
read

echo ""
echo -e "${GREEN}开始控制！（按q退出）${NC}"
echo ""

# 检查是否安装了teleop_twist_keyboard
if ros2 pkg list | grep -q "teleop_twist_keyboard"; then
    # 使用teleop_twist_keyboard
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
else
    echo -e "${YELLOW}未安装teleop_twist_keyboard，使用简化控制${NC}"
    echo ""
    echo "请手动控制："
    echo "  前进: ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}' --once"
    echo "  停止: ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' --once"
    echo ""
    echo "建议安装teleop_twist_keyboard："
    echo "  sudo apt install ros-foxy-teleop-twist-keyboard"
fi

echo ""
echo -e "${GREEN}控制已停止${NC}"