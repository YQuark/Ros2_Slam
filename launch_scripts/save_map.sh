#!/bin/bash
# ============================================
# 保存SLAM地图脚本
# ============================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_WS="/home/robot/ros2_ws"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}    保存SLAM地图${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 设置ROS2环境
source /opt/ros/foxy/setup.bash
source $ROS_WS/install/setup.bash

# 获取地图名称
MAP_NAME=${1:-robot_map_$(date +%Y%m%d_%H%M%S)}
MAP_DIR="$HOME/ros2_maps"

# 创建地图保存目录
mkdir -p "$MAP_DIR"

echo -e "${YELLOW}地图信息:${NC}"
echo "  名称: $MAP_NAME"
echo "  保存路径: $MAP_DIR/$MAP_NAME"
echo ""

# 检查SLAM服务是否运行
if ros2 service list | grep -q "/slam_toolbox/save_map"; then
    echo -e "${GREEN}正在保存地图...${NC}"
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$MAP_DIR/$MAP_NAME'}}"

    if [ $? -eq 0 ]; then
        echo ""
        echo -e "${GREEN}✓ 地图保存成功！${NC}"
        echo ""
        echo -e "${YELLOW}地图文件:${NC}"
        ls -lh "$MAP_DIR/${MAP_NAME}".* 2>/dev/null || echo "  未找到生成的地图文件"
        echo ""
        echo -e "${YELLOW}加载地图进行导航:${NC}"
        echo "  ros2 launch robot_bringup system.launch.py mode:=navigation use_camera:=true use_base:=true map_file:=$MAP_DIR/$MAP_NAME.yaml"
    else
        echo -e "${RED}✗ 地图保存失败${NC}"
    fi
else
    echo -e "${RED}✗ SLAM服务未运行${NC}"
    echo -e "${YELLOW}请先运行建图系统:${NC}"
    echo "  $SCRIPT_DIR/start_mapping.sh"
fi
