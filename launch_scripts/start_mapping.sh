#!/bin/bash
# ============================================
# 建图模式启动脚本
# 功能：camera/lidar 二选一建图 + SLAM Toolbox + RViz
# 用法：./start_mapping.sh [camera|lidar]
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
echo -e "${BLUE}    SLAM 建图模式启动${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
MAPPING_SOURCE="${1:-camera}"
if [ "$MAPPING_SOURCE" != "camera" ] && [ "$MAPPING_SOURCE" != "lidar" ]; then
    echo -e "${RED}参数错误: $MAPPING_SOURCE${NC}"
    echo "用法: $0 [camera|lidar]"
    exit 1
fi

echo -e "${YELLOW}系统组件:${NC}"
if [ "$MAPPING_SOURCE" = "camera" ]; then
    echo "  • Astra Pro 深度相机 -> depthimage_to_laserscan"
    USE_CAMERA=true
    # Prefer visual odom for camera-only mapping when available.
    if ros2 pkg list 2>/dev/null | grep -qx "rtabmap_odom"; then
        USE_VISUAL_ODOM=true
    else
        USE_VISUAL_ODOM=false
        echo -e "${YELLOW}提示: 未检测到 rtabmap_odom，回退到无视觉里程计模式${NC}"
    fi
else
    echo "  • YDLIDAR X2 激光雷达"
    USE_CAMERA=false
    USE_VISUAL_ODOM=false
fi
echo "  • SLAM Toolbox 建图引擎"
echo "  • RViz2 可视化界面"
echo ""

# 设置ROS2环境
echo -e "${YELLOW}正在设置ROS2环境...${NC}"
source /opt/ros/foxy/setup.bash
source $ROS_WS/install/setup.bash

# 清理旧会话，避免设备占用
if [ -x "$SCRIPT_DIR/stop_all.sh" ]; then
    echo -e "${YELLOW}清理旧ROS2会话...${NC}"
    "$SCRIPT_DIR/stop_all.sh" >/dev/null 2>&1 || true
    sleep 1
fi

if [ "$MAPPING_SOURCE" = "lidar" ]; then
    echo ""
    echo -e "${YELLOW}检查雷达设备...${NC}"
    if [ -e /dev/ttyUSB0 ]; then
        echo -e "${GREEN}✓ 找到雷达设备: /dev/ttyUSB0${NC}"
    else
        echo -e "${RED}✗ 未找到雷达设备 /dev/ttyUSB0${NC}"
        echo -e "${YELLOW}请检查雷达连接${NC}"
        exit 1
    fi
fi

# 启动建图系统
echo ""
echo -e "${GREEN}正在启动统一建图系统...${NC}"
echo -e "${YELLOW}入口: robot_bringup/system.launch.py${NC}"
echo -e "${YELLOW}建图源: ${MAPPING_SOURCE}${NC}"
echo -e "${YELLOW}地图分辨率: 0.05m${NC}"
echo -e "${YELLOW}激光范围: 12.0m${NC}"
echo ""
echo -e "${YELLOW}操作提示:${NC}"
echo "  • 使用键盘控制机器人移动进行建图"
echo "  • 在RViz中查看实时地图"
echo "  • 按 Ctrl+C 停止建图"
echo ""
echo -e "${YELLOW}保存地图: 运行 $SCRIPT_DIR/save_map.sh${NC}"
echo ""

ros2 launch robot_bringup system.launch.py \
    mode:=mapping \
    mapping_source:=${MAPPING_SOURCE} \
    use_camera:=${USE_CAMERA} \
    use_base:=false \
    use_visual_odom:=${USE_VISUAL_ODOM} \
    use_rviz:=true \
    camera_use_uvc:=true \
    camera_enable_ir:=false \
    camera_color_info_url:=file:///home/robot/.ros/camera_info/rgb_Astra_Orbbec.yaml \
    slam_params_file:=/home/robot/ros2_ws/src/robot_bringup/config/slam_toolbox_mapping_fast.yaml

echo ""
echo -e "${GREEN}SLAM建图系统已停止${NC}"
