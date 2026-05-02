#!/bin/bash
# ============================================
# 建图链路验收脚本（运行于建图过程中）
# 检查项：
# 1) /scan 频率
# 2) /map 发布
# 3) map->odom TF
# 4) RViz 关键订阅（/map_points 或 /map）
# ============================================

set -o pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ROS_WS="/home/robot/ros2_ws"
MIN_SCAN_HZ=6.0
CHECK_SECONDS=6
ALLOW_HEADLESS=false

usage() {
    echo "用法: $0 [--allow-headless] [--min-scan-hz N]"
    echo "  --allow-headless   不要求 RViz 订阅检查通过"
    echo "  --min-scan-hz N    /scan 最低频率阈值（默认 ${MIN_SCAN_HZ}）"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --allow-headless)
            ALLOW_HEADLESS=true
            shift
            ;;
        --min-scan-hz)
            MIN_SCAN_HZ="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo -e "${RED}参数错误: $1${NC}"
            usage
            exit 1
            ;;
    esac
done

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    建图链路验收${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"

fail() {
    echo -e "${RED}✗ $1${NC}"
    exit 1
}

pass() {
    echo -e "${GREEN}✓ $1${NC}"
}

if ! ros2 topic list | grep -qx "/scan"; then
    fail "未检测到 /scan 话题"
fi

SCAN_INFO="$(ros2 topic info /scan -v 2>/dev/null || true)"
if ! echo "$SCAN_INFO" | grep -q "Publisher count: [1-9]"; then
    fail "/scan 没有发布者"
fi

SCAN_METRICS="$(python3 - "$CHECK_SECONDS" <<'PY'
import sys
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

duration = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0

rclpy.init()
node = rclpy.create_node("mapping_pipeline_scan_rate")
state = {"count": 0, "first": None, "last": None}

def cb(msg):
    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if state["first"] is None:
        state["first"] = t
    state["last"] = t
    state["count"] += 1

node.create_subscription(LaserScan, "/scan", cb, qos_profile_sensor_data)
start = time.time()
while time.time() - start < duration:
    rclpy.spin_once(node, timeout_sec=0.2)

count = state["count"]
if count <= 1 or state["first"] is None or state["last"] is None:
    hz = 0.0
else:
    dt = state["last"] - state["first"]
    hz = ((count - 1) / dt) if dt > 0 else 0.0

print(f"COUNT={count}")
print(f"HZ={hz:.3f}")
node.destroy_node()
rclpy.shutdown()
PY
)"

SCAN_COUNT="$(echo "$SCAN_METRICS" | awk -F= '/^COUNT=/{print $2}' | tr -d '[:space:]')"
SCAN_HZ="$(echo "$SCAN_METRICS" | awk -F= '/^HZ=/{print $2}' | tr -d '[:space:]')"

if [ -z "$SCAN_COUNT" ] || [ "${SCAN_COUNT:-0}" -lt 2 ]; then
    fail "/scan 数据不足（消息数 < 2）"
fi
if [ -z "$SCAN_HZ" ]; then
    fail "未获取到 /scan 频率"
fi
if ! awk "BEGIN {exit !($SCAN_HZ >= $MIN_SCAN_HZ)}"; then
    fail "/scan 频率过低 (${SCAN_HZ} Hz < ${MIN_SCAN_HZ} Hz)"
fi
pass "/scan 正常，频率 ${SCAN_HZ} Hz"

MAP_INFO="$(ros2 topic info /map -v 2>/dev/null || true)"
if ! echo "$MAP_INFO" | grep -q "Publisher count: [1-9]"; then
    fail "/map 没有发布者（slam_toolbox 未正常发布）"
fi
pass "/map 发布正常"

if ! python3 - <<'PY'
import time
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node("mapping_pipeline_tf_check")
buf = Buffer()
listener = TransformListener(buf, node)

ok = False
end = time.time() + 6.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        buf.lookup_transform("map", "odom", Time(), timeout=Duration(seconds=0.2))
        ok = True
        break
    except Exception:
        pass

node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ok else 1)
PY
then
    fail "未检测到 map->odom TF"
fi
pass "map->odom TF 正常"

RVIZ_OK=false
MAP_POINTS_INFO="$(ros2 topic info /map_points -v 2>/dev/null || true)"
if echo "$MAP_POINTS_INFO" | grep -q "Node name: rviz2"; then
    RVIZ_OK=true
fi

if [ "$RVIZ_OK" = false ]; then
    if echo "$MAP_INFO" | grep -q "Node name: rviz2"; then
        RVIZ_OK=true
    fi
fi

if [ "$RVIZ_OK" = true ]; then
    pass "RViz 订阅状态正常"
else
    if [ "$ALLOW_HEADLESS" = true ]; then
        echo -e "${YELLOW}! 未检测到 RViz 订阅（headless 已允许）${NC}"
    else
        fail "未检测到 RViz 对 /map_points 或 /map 的订阅"
    fi
fi

echo ""
echo -e "${GREEN}全部检查通过${NC}"
exit 0
