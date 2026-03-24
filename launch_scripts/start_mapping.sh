#!/bin/bash
# ============================================
# 建图模式启动脚本
# 功能：camera/lidar 二选一建图 + SLAM Toolbox + RViz
# 用法：./start_mapping.sh [camera|lidar] [auto|quality|precision|fast] [--skip-lidar-check] [--no-rviz] [--real-base] [--base-port PORT]
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
LIDAR_HEALTH_SCRIPT="$SCRIPT_DIR/check_lidar_health.sh"
DETECT_BASE_PORT_SCRIPT="$SCRIPT_DIR/detect_base_port.sh"
DETECT_LIDAR_PORT_SCRIPT="$SCRIPT_DIR/detect_lidar_port.sh"
DEFAULT_LIDAR_PARAMS="$ROS_WS/src/robot_bringup/config/slam_toolbox_mapping.yaml"
PRECISION_PARAMS="$ROS_WS/src/robot_bringup/config/slam_toolbox_mapping_lidar_precision.yaml"
FAST_PARAMS="$ROS_WS/src/robot_bringup/config/slam_toolbox_mapping_fast.yaml"
MAPPING_LIDAR_DEVICE_PARAMS="$ROS_WS/src/robot_bringup/config/ydlidar_X2_mapping.yaml"
DEFAULT_LIDAR_DEVICE_PARAMS="$ROS_WS/src/ydlidar_ros2_driver/params/X2.yaml"
LIDAR_RVIZ_CONFIG="$ROS_WS/src/robot_bringup/rviz/lidar_mapping.rviz"
DEFAULT_RVIZ_CONFIG="$ROS_WS/src/robot_bringup/rviz/system.rviz"
BASE_PORT="${BASE_PORT:-auto}"
TMP_LIDAR_PARAMS_FILE=""

cleanup() {
    if [ -n "$TMP_LIDAR_PARAMS_FILE" ] && [ -f "$TMP_LIDAR_PARAMS_FILE" ]; then
        rm -f "$TMP_LIDAR_PARAMS_FILE"
    fi
}

trap cleanup EXIT

resolve_port() {
    local candidate="$1"
    readlink -f "$candidate" 2>/dev/null || printf '%s\n' "$candidate"
}

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

infer_lidar_port() {
    local base_port="$1"
    local resolved_base
    local port
    local resolved_port
    local candidates=()

    resolved_base="$(resolve_port "$base_port")"

    for port in /dev/ttyUSB*; do
        [ -e "$port" ] || continue
        resolved_port="$(resolve_port "$port")"
        if [ "$resolved_port" != "$resolved_base" ]; then
            candidates+=("$resolved_port")
        fi
    done

    if [ "${#candidates[@]}" -eq 1 ]; then
        printf '%s\n' "${candidates[0]}"
    fi
}

usage() {
    echo "用法: $0 [camera|lidar] [auto|quality|precision|fast] [--skip-lidar-check] [--no-rviz] [--real-base] [--base-port PORT]"
    echo "  camera|lidar: 建图源，默认 camera"
    echo "  auto|quality|precision|fast: SLAM 参数档位，默认 auto（lidar->quality, camera->fast）"
    echo "  --skip-lidar-check: 跳过雷达健康检查（仅 lidar 模式有效）"
    echo "  --no-rviz: 不启动 RViz（适用于显卡渲染异常场景）"
    echo "  --real-base: 接入真实底盘里程计（仅在底盘已连接时使用）"
    echo "  --base-port PORT: 指定底盘串口，默认 auto"
}

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    SLAM 建图模式启动${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

MAPPING_SOURCE="camera"
SLAM_PROFILE="auto"
SKIP_LIDAR_CHECK=false
USE_RVIZ=true
USE_REAL_BASE=false
HAS_SOURCE=false
HAS_PROFILE=false

while [ $# -gt 0 ]; do
    arg="$1"
    case "$arg" in
        camera|lidar)
            if [ "$HAS_SOURCE" = true ]; then
                echo -e "${RED}参数错误: 重复指定建图源 '$arg'${NC}"
                usage
                exit 1
            fi
            MAPPING_SOURCE="$arg"
            HAS_SOURCE=true
            shift
            ;;
        auto|quality|precision|fast)
            if [ "$HAS_PROFILE" = true ]; then
                echo -e "${RED}参数错误: 重复指定参数档位 '$arg'${NC}"
                usage
                exit 1
            fi
            SLAM_PROFILE="$arg"
            HAS_PROFILE=true
            shift
            ;;
        --skip-lidar-check)
            SKIP_LIDAR_CHECK=true
            shift
            ;;
        --no-rviz)
            USE_RVIZ=false
            shift
            ;;
        --real-base)
            USE_REAL_BASE=true
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
            echo -e "${RED}参数错误: $arg${NC}"
            usage
            exit 1
            ;;
    esac
done

# 设置ROS2环境
echo -e "${YELLOW}正在设置ROS2环境...${NC}"
source /opt/ros/foxy/setup.bash
source "$ROS_WS/install/setup.bash"

if [ "$SLAM_PROFILE" = "auto" ]; then
    if [ "$MAPPING_SOURCE" = "lidar" ]; then
        SLAM_PROFILE="quality"
    else
        SLAM_PROFILE="fast"
    fi
fi

if [ "$SLAM_PROFILE" = "quality" ]; then
    SLAM_PARAMS_FILE="$DEFAULT_LIDAR_PARAMS"
elif [ "$SLAM_PROFILE" = "precision" ]; then
    SLAM_PARAMS_FILE="$PRECISION_PARAMS"
else
    SLAM_PARAMS_FILE="$FAST_PARAMS"
fi

if [ "$MAPPING_SOURCE" = "lidar" ] && [ -f "$LIDAR_RVIZ_CONFIG" ]; then
    RVIZ_CONFIG_FILE="$LIDAR_RVIZ_CONFIG"
else
    RVIZ_CONFIG_FILE="$DEFAULT_RVIZ_CONFIG"
fi

if [ -f "$MAPPING_LIDAR_DEVICE_PARAMS" ]; then
    LIDAR_DEVICE_PARAMS_FILE="$MAPPING_LIDAR_DEVICE_PARAMS"
else
    LIDAR_DEVICE_PARAMS_FILE="$DEFAULT_LIDAR_DEVICE_PARAMS"
fi

BASE_MODE="none"
USE_BASE_ARG="false"
if [ "$USE_REAL_BASE" = true ]; then
    BASE_MODE="real"
    USE_BASE_ARG="true"
    if [ "$BASE_PORT" = "auto" ] && [ -x "$DETECT_BASE_PORT_SCRIPT" ]; then
        DETECTED_BASE_PORT="$($DETECT_BASE_PORT_SCRIPT)"
        if [ -n "$DETECTED_BASE_PORT" ]; then
            BASE_PORT="$DETECTED_BASE_PORT"
        fi
    fi
fi

if [ "$MAPPING_SOURCE" = "lidar" ]; then
    CONFIGURED_LIDAR_PORT="$(get_yaml_port "$LIDAR_DEVICE_PARAMS_FILE" || true)"
    DETECTED_LIDAR_PORT=""

    if [ -x "$DETECT_LIDAR_PORT_SCRIPT" ]; then
        DETECTED_LIDAR_PORT="$($DETECT_LIDAR_PORT_SCRIPT "$CONFIGURED_LIDAR_PORT" 2>/dev/null || true)"
    elif [ "$USE_REAL_BASE" = true ] && [ "$BASE_PORT" != "auto" ]; then
        DETECTED_LIDAR_PORT="$(infer_lidar_port "$BASE_PORT" || true)"
    fi

    if [ -n "$DETECTED_LIDAR_PORT" ] && [ "$DETECTED_LIDAR_PORT" != "$CONFIGURED_LIDAR_PORT" ]; then
        TMP_LIDAR_PARAMS_FILE="$(mktemp /tmp/ydlidar_mapping_XXXX.yaml)"
        render_lidar_params_with_port "$LIDAR_DEVICE_PARAMS_FILE" "$DETECTED_LIDAR_PORT" "$TMP_LIDAR_PARAMS_FILE"
        LIDAR_DEVICE_PARAMS_FILE="$TMP_LIDAR_PARAMS_FILE"
    fi
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
if [ "$USE_REAL_BASE" = true ]; then
    echo "  • STM32 底盘里程计 (${BASE_PORT})"
else
    echo "  • 底盘里程计已禁用"
fi
echo "  • SLAM Toolbox 建图引擎"
if [ "$USE_RVIZ" = true ]; then
    echo "  • RViz2 可视化界面"
else
    echo "  • RViz2 已禁用（--no-rviz）"
fi
echo ""

# 清理旧会话，避免设备占用
if [ -x "$SCRIPT_DIR/stop_all.sh" ]; then
    echo -e "${YELLOW}清理旧ROS2会话...${NC}"
    "$SCRIPT_DIR/stop_all.sh" >/dev/null 2>&1 || true
    sleep 1
fi

if [ "$MAPPING_SOURCE" = "lidar" ]; then
    echo ""
    echo -e "${YELLOW}检查雷达设备...${NC}"
    CURRENT_LIDAR_PORT="$(get_yaml_port "$LIDAR_DEVICE_PARAMS_FILE" || true)"
    if [ -n "$CURRENT_LIDAR_PORT" ] && [ -e "$CURRENT_LIDAR_PORT" ]; then
        echo -e "${GREEN}✓ 找到雷达设备: ${CURRENT_LIDAR_PORT}${NC}"
    else
        echo -e "${RED}✗ 未找到雷达设备${NC}"
        echo -e "${YELLOW}请检查雷达连接${NC}"
        exit 1
    fi

    if [ "$SKIP_LIDAR_CHECK" = false ] && [ -x "$LIDAR_HEALTH_SCRIPT" ]; then
        echo -e "${YELLOW}执行雷达数据健康检查...${NC}"
        "$LIDAR_HEALTH_SCRIPT" "$LIDAR_DEVICE_PARAMS_FILE" || {
            echo -e "${RED}✗ 雷达健康检查未通过，已终止建图启动${NC}"
            echo -e "${YELLOW}可手动跳过检查: $0 lidar $SLAM_PROFILE --skip-lidar-check${NC}"
            exit 1
        }
    fi
fi

# 启动建图系统
echo ""
echo -e "${GREEN}正在启动统一建图系统...${NC}"
echo -e "${YELLOW}入口: robot_bringup/system.launch.py${NC}"
echo -e "${YELLOW}建图源: ${MAPPING_SOURCE}${NC}"
echo -e "${YELLOW}参数档位: ${SLAM_PROFILE}${NC}"
echo -e "${YELLOW}SLAM参数: ${SLAM_PARAMS_FILE}${NC}"
echo -e "${YELLOW}雷达参数: ${LIDAR_DEVICE_PARAMS_FILE}${NC}"
echo -e "${YELLOW}底盘模式: ${BASE_MODE}${NC}"
if [ "$USE_REAL_BASE" = true ]; then
    echo -e "${YELLOW}底盘串口: ${BASE_PORT}${NC}"
fi
if [ "$MAPPING_SOURCE" = "lidar" ]; then
    echo -e "${YELLOW}雷达串口: $(get_yaml_port "$LIDAR_DEVICE_PARAMS_FILE")${NC}"
fi
echo -e "${YELLOW}RViz配置: ${RVIZ_CONFIG_FILE}${NC}"
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
    base_mode:=${BASE_MODE} \
    use_base:=${USE_BASE_ARG} \
    base_port:=${BASE_PORT} \
    use_visual_odom:=${USE_VISUAL_ODOM} \
    use_rviz:=${USE_RVIZ} \
    lidar_params_file:=${LIDAR_DEVICE_PARAMS_FILE} \
    rviz_config:=${RVIZ_CONFIG_FILE} \
    camera_use_uvc:=true \
    camera_enable_ir:=false \
    camera_color_info_url:=file:///home/robot/ros2_ws/src/robot_bringup/config/camera_info/rgb_Astra_Orbbec.yaml \
    slam_params_file:=${SLAM_PARAMS_FILE}

echo ""
echo -e "${GREEN}SLAM建图系统已停止${NC}"
