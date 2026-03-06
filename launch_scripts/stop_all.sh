#!/bin/bash
# ============================================
# 停止所有ROS2节点脚本
# ============================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}    停止所有ROS2节点${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 设置ROS2环境
source /opt/ros/foxy/setup.bash

# 停止tmux会话
if tmux has-session -t robot_system 2>/dev/null; then
    echo -e "${YELLOW}正在停止tmux会话...${NC}"
    tmux kill-session -t robot_system
    echo -e "${GREEN}✓ tmux会话已停止${NC}"
else
    echo -e "${YELLOW}未找到robot_system会话${NC}"
fi

# 停止所有ROS2节点
echo -e "${YELLOW}正在停止ROS2节点...${NC}"
ros2 node list 2>/dev/null | while read -r node; do
    echo -e "${YELLOW}  停止: $node${NC}"
    ros2 lifecycle set "$node" deactivate 2>/dev/null
    ros2 lifecycle set "$node" cleanup 2>/dev/null
done

# 强制终止
sleep 1
pkill -f "ros2 run" 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null
pkill -f "astra_camera_node" 2>/dev/null
pkill -f "depthimage_to_laserscan_node" 2>/dev/null
pkill -f "rgbd_odometry" 2>/dev/null
pkill -f "slam_toolbox" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
pkill -f "rtabmap" 2>/dev/null

echo -e "${GREEN}✓ 所有节点已停止${NC}"
echo ""
