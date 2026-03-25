#!/bin/bash
# ============================================
# 导航模式启动脚本
# 功能：雷达 + (真实底盘/虚拟底盘) + 定位 + Nav2 + RViz
# 用法：./start_navigation.sh [map.yaml] [--fake-base] [--real-base] [--no-rviz] [--skip-lidar-check] [--base-port PORT]
# ============================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ROS_WS="/home/robot/ros2_ws"
DEFAULT_MAP="/home/robot/ros2_maps/latest.yaml"
DEFAULT_LIDAR_DEVICE_PARAMS="$ROS_WS/src/robot_bringup/config/ydlidar_X2_mapping.yaml"
MAP_FILE="$DEFAULT_MAP"
BASE_MODE="real"
USE_RVIZ=true
SKIP_LIDAR_CHECK=false
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BASE_PORT="${BASE_PORT:-auto}"
LIDAR_HEALTH_SCRIPT="$SCRIPT_DIR/check_lidar_health.sh"
DETECT_LIDAR_PORT_SCRIPT="$SCRIPT_DIR/detect_lidar_port.sh"
TMP_LIDAR_PARAMS_FILE=""

cleanup() {
    if [ -n "$TMP_LIDAR_PARAMS_FILE" ] && [ -f "$TMP_LIDAR_PARAMS_FILE" ]; then
        rm -f "$TMP_LIDAR_PARAMS_FILE"
    fi
}

trap cleanup EXIT

get_yaml_port() {
    local yaml_file="$1"
    awk '
        $1 == "port:" {
            print $2
            exit
        }
    ' "$yaml_file"
}

render_lidar_params_with_port() {
    local src="$1"
    local port="$2"
    local dst="$3"

    sed -E "s|^([[:space:]]*port:).*|\\1 ${port}|" "$src" > "$dst"
}

usage() {
    echo "用法: $0 [map.yaml] [--fake-base] [--real-base] [--no-rviz] [--skip-lidar-check] [--base-port PORT]"
    echo "  map.yaml      地图文件路径（默认: $DEFAULT_MAP）"
    echo "  --fake-base   使用虚拟底盘（发布 /odom 与 odom->base_link）"
    echo "  --real-base   使用真实底盘（默认）"
    echo "  --no-rviz     不启动 RViz"
    echo "  --skip-lidar-check  跳过雷达健康检查"
    echo "  --base-port PORT    指定底盘串口，默认 auto"
}

while [ $# -gt 0 ]; do
    arg="$1"
    case "$arg" in
        --fake-base)
            BASE_MODE="fake"
            shift
            ;;
        --real-base)
            BASE_MODE="real"
            shift
            ;;
        --no-rviz)
            USE_RVIZ=false
            shift
            ;;
        --skip-lidar-check)
            SKIP_LIDAR_CHECK=true
            shift
            ;;
        --base-port)
            if [ $# -lt 2 ]; then
                echo -e "${RED}参数错误: --base-port 需要一个串口路径${NC}"
                usage
                exit 1
            fi
            BASE_PORT="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            if [[ "$arg" == --* ]]; then
                echo -e "${RED}参数错误: $arg${NC}"
                usage
                exit 1
            fi
            MAP_FILE="$arg"
            shift
            ;;
    esac
done

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    导航模式启动${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

source /opt/ros/foxy/setup.bash
source $ROS_WS/install/setup.bash

if [ "$BASE_PORT" = "auto" ] && [ -x "$SCRIPT_DIR/detect_base_port.sh" ]; then
    DETECTED_BASE_PORT="$($SCRIPT_DIR/detect_base_port.sh)"
    if [ -n "$DETECTED_BASE_PORT" ]; then
        BASE_PORT="$DETECTED_BASE_PORT"
    fi
fi

LIDAR_PARAMS_FILE="$DEFAULT_LIDAR_DEVICE_PARAMS"
CONFIGURED_LIDAR_PORT="$(get_yaml_port "$LIDAR_PARAMS_FILE" || true)"
if [ -x "$DETECT_LIDAR_PORT_SCRIPT" ]; then
    DETECTED_LIDAR_PORT="$($DETECT_LIDAR_PORT_SCRIPT "$CONFIGURED_LIDAR_PORT" 2>/dev/null || true)"
    if [ -n "$DETECTED_LIDAR_PORT" ] && [ "$DETECTED_LIDAR_PORT" != "$CONFIGURED_LIDAR_PORT" ]; then
        TMP_LIDAR_PARAMS_FILE="$(mktemp /tmp/ydlidar_nav_XXXX.yaml)"
        render_lidar_params_with_port "$LIDAR_PARAMS_FILE" "$DETECTED_LIDAR_PORT" "$TMP_LIDAR_PARAMS_FILE"
        LIDAR_PARAMS_FILE="$TMP_LIDAR_PARAMS_FILE"
    fi
fi

if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}✗ 地图文件不存在: $MAP_FILE${NC}"
    usage
    exit 1
fi

CURRENT_LIDAR_PORT="$(get_yaml_port "$LIDAR_PARAMS_FILE" || true)"
if [ -z "$CURRENT_LIDAR_PORT" ] || [ ! -e "$CURRENT_LIDAR_PORT" ]; then
    echo -e "${RED}✗ 未找到雷达设备${NC}"
    exit 1
fi

if [ "$SKIP_LIDAR_CHECK" = false ] && [ -x "$LIDAR_HEALTH_SCRIPT" ]; then
    echo -e "${YELLOW}执行雷达数据健康检查...${NC}"
    "$LIDAR_HEALTH_SCRIPT" "$LIDAR_PARAMS_FILE" || {
        echo -e "${RED}✗ 雷达健康检查未通过，已终止导航启动${NC}"
        echo -e "${YELLOW}可手动跳过检查: $0 $MAP_FILE --skip-lidar-check${NC}"
        exit 1
    }
fi

if [ "$BASE_MODE" = "real" ]; then
    if [ "$BASE_PORT" = "auto" ]; then
        echo -e "${YELLOW}⚠ 未检测到底盘 CP2102 串口，桥接节点将以 auto 模式持续重试${NC}"
        echo -e "${YELLOW}如暂不接底盘，也可改用虚拟底盘: $0 $MAP_FILE --fake-base${NC}"
    fi
fi

echo -e "${GREEN}✓ 使用地图: $MAP_FILE${NC}"
echo -e "${YELLOW}雷达串口: ${CURRENT_LIDAR_PORT}${NC}"
echo -e "${YELLOW}底盘模式: ${BASE_MODE}${NC}"
if [ "$BASE_MODE" = "real" ]; then
    echo -e "${YELLOW}底盘串口: ${BASE_PORT}${NC}"
fi
if [ "$USE_RVIZ" = true ]; then
    echo -e "${YELLOW}RViz: 开启${NC}"
else
    echo -e "${YELLOW}RViz: 关闭${NC}"
fi
echo -e "${YELLOW}按 Ctrl+C 停止${NC}"
echo ""

ros2 launch robot_bringup system.launch.py \
    mode:=navigation \
    use_camera:=false \
    use_lidar:=true \
    base_mode:=${BASE_MODE} \
    use_base:=true \
    use_rviz:=${USE_RVIZ} \
    map_file:=$MAP_FILE \
    base_port:=${BASE_PORT} \
    base_baudrate:=115200 \
    lidar_params_file:=${LIDAR_PARAMS_FILE}
