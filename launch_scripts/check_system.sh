#!/bin/bash
# ============================================
# 系统诊断脚本
# ============================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BASE_PORT=""

if [ -x "$SCRIPT_DIR/detect_base_port.sh" ]; then
    BASE_PORT="$($SCRIPT_DIR/detect_base_port.sh)"
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    系统诊断工具${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 1. 检查ROS2环境
echo -e "${BLUE}1. ROS2 环境${NC}"
if [ -n "$ROS_DISTRO" ]; then
    echo -e "${GREEN}✓ ROS2版本: $ROS_DISTRO${NC}"
    echo -e "${GREEN}✓ ROS_DOMAIN_ID: $ROS_DOMAIN_ID${NC}"
else
    echo -e "${RED}✗ ROS2环境未设置${NC}"
fi
echo ""

# 2. 检查工作空间
echo -e "${BLUE}2. 工作空间${NC}"
if [ -f /home/robot/ros2_ws/install/setup.bash ]; then
    echo -e "${GREEN}✓ 工作空间已构建${NC}"
    source /home/robot/ros2_ws/install/setup.bash 2>/dev/null
    ros2 pkg list | grep -E "ydlidar|astra|robot" | while read pkg; do
        echo -e "${GREEN}  ✓ $pkg${NC}"
    done
else
    echo -e "${RED}✗ 工作空间未构建${NC}"
fi
echo ""

# 3. 检查硬件设备
echo -e "${BLUE}3. 硬件设备${NC}"

# 雷达
if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}✓ 雷达: /dev/ttyUSB0${NC}"
    ls -l /dev/ttyUSB0 | awk '{print "  权限: " $1 " " $3}'
else
    echo -e "${RED}✗ 雷达: /dev/ttyUSB0 不存在${NC}"
fi

# 摄像头
if [ -e /dev/video0 ]; then
    echo -e "${GREEN}✓ 摄像头: /dev/video0${NC}"
else
    echo -e "${YELLOW}⚠ 摄像头: /dev/video0 不存在（可选）${NC}"
fi

# 下位机
if [ -n "$BASE_PORT" ] && [ -e "$BASE_PORT" ]; then
    echo -e "${GREEN}✓ 下位机: ${BASE_PORT}${NC}"
else
    echo -e "${YELLOW}⚠ 下位机: 未检测到 CP2102 串口（当前可保持未连接）${NC}"
fi
echo ""

# 4. 检查运行中的节点
echo -e "${BLUE}4. 运行中的节点${NC}"
NODES=$(ros2 node list 2>/dev/null)
if [ -n "$NODES" ]; then
    echo -e "${GREEN}✓ 运行中的节点:${NC}"
    echo "$NODES" | while read node; do
        echo -e "${GREEN}  • $node${NC}"
    done
else
    echo -e "${YELLOW}⚠ 没有运行中的节点${NC}"
fi
echo ""

# 5. 检查话题
echo -e "${BLUE}5. 活动话题${NC}"
TOPICS=$(ros2 topic list 2>/dev/null)
if [ -n "$TOPICS" ]; then
    echo -e "${GREEN}✓ 活动话题:${NC}"
    echo "$TOPICS" | head -10 | while read topic; do
        TYPE=$(ros2 topic info $topic 2>/dev/null | grep Type | awk '{print $2}')
        echo -e "${GREEN}  • $topic ($TYPE)${NC}"
    done
else
    echo -e "${YELLOW}⚠ 没有活动话题${NC}"
fi
echo ""

# 6. 检查地图
echo -e "${BLUE}6. 已保存的地图${NC}"
if [ -d "$HOME/ros2_maps" ]; then
    MAP_COUNT=$(ls -1 $HOME/ros2_maps/*.posegraph 2>/dev/null | wc -l)
    if [ $MAP_COUNT -gt 0 ]; then
        echo -e "${GREEN}✓ 找到 $MAP_COUNT 个地图${NC}"
        ls -1t $HOME/ros2_maps/*.posegraph 2>/dev/null | head -5 | while read map; do
            MAP_NAME=$(basename $map .posegraph)
            MAP_SIZE=$(du -h "$map" | awk '{print $1}')
            echo -e "${GREEN}  • $MAP_NAME ($MAP_SIZE)${NC}"
        done
    else
        echo -e "${YELLOW}⚠ 没有已保存的地图${NC}"
    fi
else
    echo -e "${YELLOW}⚠ 地图目录不存在: $HOME/ros2_maps${NC}"
fi
echo ""

# 7. 系统资源
echo -e "${BLUE}7. 系统资源${NC}"
CPU=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
MEM=$(free -m | grep Mem | awk '{printf "%.1f%%", $3/$2*100}')
echo -e "${GREEN}  CPU使用率: ${CPU}%${NC}"
echo -e "${GREEN}  内存使用率: ${MEM}${NC}"
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}诊断完成${NC}"
echo -e "${BLUE}========================================${NC}"
