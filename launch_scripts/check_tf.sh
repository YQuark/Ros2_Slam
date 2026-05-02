#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/home/robot/ros2_ws/launch_scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

print_header "TF 树诊断"

# 1. 检查 TF 话题
echo -e "${BLUE}1. TF 话题${NC}"
if ros2 topic list 2>/dev/null | grep -q "/tf"; then
    echo -e "${GREEN}✓ /tf 话题存在${NC}"
else
    echo -e "${RED}✗ /tf 话题不存在${NC}"
fi
if ros2 topic list 2>/dev/null | grep -q "/tf_static"; then
    echo -e "${GREEN}✓ /tf_static 话题存在${NC}"
else
    echo -e "${YELLOW}⚠ /tf_static 话题不存在${NC}"
fi
echo ""

# 2. 检查关键 TF 帧
echo -e "${BLUE}2. 关键 TF 帧${NC}"
EXPECTED_TFS=(
    "odom->base_link"
    "base_link->base_footprint"
    "base_link->laser_frame"
    "base_link->imu_link"
)
for tf_pair in "${EXPECTED_TFS[@]}"; do
    parent="${tf_pair%->*}"
    child="${tf_pair#*->}"
    if ros2 run tf2_ros tf2_monitor --no-daemon 2>/dev/null | grep -q "${child}" 2>/dev/null; then
        echo -e "${GREEN}✓ ${parent} -> ${child}${NC}"
    else
        echo -e "${YELLOW}⚠ ${parent} -> ${child} (未检测到)${NC}"
    fi
done
echo ""

# 3. 快速 TF 链检查 (使用 tf2_echo，超时 2s)
echo -e "${BLUE}3. TF 链路检查${NC}"
setup_ros_env 2>/dev/null || true

for tf_pair in "odom base_link" "base_link laser_frame" "base_link imu_link"; do
    parent=$(echo "$tf_pair" | awk '{print $1}')
    child=$(echo "$tf_pair" | awk '{print $2}')
    if command -v timeout >/dev/null 2>&1; then
        result=$(timeout 2s ros2 run tf2_ros tf2_echo "$parent" "$child" --no-daemon 2>/dev/null | head -3 || true)
    else
        result=""
    fi
    if [ -n "$result" ]; then
        echo -e "${GREEN}✓ ${parent} -> ${child}: 可达${NC}"
    else
        echo -e "${YELLOW}⚠ ${parent} -> ${child}: 超时或不可达${NC}"
    fi
done
echo ""

echo -e "${GREEN}TF 诊断完成${NC}"
echo "  如需详细 TF 树，运行: ros2 run tf2_tools view_frames"
