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
WASD_TELEOP_SCRIPT="$SCRIPT_DIR/wasd_teleop.py"

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
echo "  w : 前进"
echo "  s : 后退"
echo "  a : 左转"
echo "  d : 右转"
echo "  x / 空格 : 停止"
echo "  q / z : 增减线速度上限"
echo "  e / c : 增减角速度"
echo "  Ctrl+C : 退出"
echo ""
echo -e "${YELLOW}按任意键继续...${NC}"
read

echo ""
echo -e "${GREEN}开始控制！（按 Ctrl+C 退出）${NC}"
echo ""

if [ -x "$WASD_TELEOP_SCRIPT" ]; then
    python3 "$WASD_TELEOP_SCRIPT"
else
    echo -e "${YELLOW}未找到 WASD 遥控脚本: $WASD_TELEOP_SCRIPT${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}控制已停止${NC}"
