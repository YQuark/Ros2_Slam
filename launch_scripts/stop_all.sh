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
pkill -f "bridge_node" 2>/dev/null
pkill -f "stm32_robot_bridge" 2>/dev/null
pkill -f "ydlidar_ros2_driver_node" 2>/dev/null
pkill -f "odom_simulator" 2>/dev/null
pkill -f "fake_base_odom.py" 2>/dev/null
pkill -f "fake_base_odom" 2>/dev/null
pkill -f "frontier_explorer.py" 2>/dev/null
pkill -f "planner_server" 2>/dev/null
pkill -f "controller_server" 2>/dev/null
pkill -f "recoveries_server" 2>/dev/null
pkill -f "bt_navigator" 2>/dev/null
pkill -f "waypoint_follower" 2>/dev/null
pkill -f "lifecycle_manager_navigation" 2>/dev/null
pkill -f "lifecycle_manager_localization" 2>/dev/null
pkill -f "map_server" 2>/dev/null
pkill -f "amcl" 2>/dev/null
pkill -f "async_slam_toolbox_node" 2>/dev/null
pkill -f "map_to_pointcloud_viz.py" 2>/dev/null
pkill -f "static_transform_publisher" 2>/dev/null
pkill -f "astra_camera_node" 2>/dev/null
pkill -f "depthimage_to_laserscan_node" 2>/dev/null
pkill -f "rgbd_odometry" 2>/dev/null
pkill -f "slam_toolbox" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
pkill -f "rtabmap" 2>/dev/null

# 清理 ROS2 CLI daemon，避免 node graph 残留旧节点名
ros2 daemon stop >/dev/null 2>&1 || true

echo -e "${GREEN}✓ 所有节点已停止${NC}"
echo ""
