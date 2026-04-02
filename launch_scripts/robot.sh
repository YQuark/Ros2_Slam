#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="/home/robot/ros2_ws"

# shellcheck source=/home/robot/ros2_ws/launch_scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

LIDAR_HEALTH_SCRIPT="${SCRIPT_DIR}/check_lidar_health.sh"
DETECT_BASE_PORT_SCRIPT="${SCRIPT_DIR}/detect_base_port.sh"
DETECT_LIDAR_PORT_SCRIPT="${SCRIPT_DIR}/detect_lidar_port.sh"
STOP_ALL_SCRIPT="${SCRIPT_DIR}/stop_all.sh"
SAVE_MAP_SCRIPT="${SCRIPT_DIR}/save_map.sh"
CHECK_SYSTEM_SCRIPT="${SCRIPT_DIR}/check_system.sh"
CHECK_MAPPING_SCRIPT="${SCRIPT_DIR}/check_mapping_pipeline.sh"
KEYBOARD_CONTROL_SCRIPT="${SCRIPT_DIR}/keyboard_control.sh"

DEFAULT_MAP="/home/robot/ros2_maps/latest.yaml"
DEFAULT_SLAM_PARAMS="${ROS_WS}/src/robot_bringup/config/slam_toolbox_mapping.yaml"
PRECISION_SLAM_PARAMS="${ROS_WS}/src/robot_bringup/config/slam_toolbox_mapping_lidar_precision.yaml"
FAST_SLAM_PARAMS="${ROS_WS}/src/robot_bringup/config/slam_toolbox_mapping_fast.yaml"
DEFAULT_LIDAR_PARAMS="${ROS_WS}/src/robot_bringup/config/ydlidar_X2_mapping.yaml"
DEFAULT_LIDAR_FALLBACK_PARAMS="${ROS_WS}/src/ydlidar_ros2_driver/params/X2.yaml"
DEFAULT_LIDAR_RVIZ="${ROS_WS}/src/robot_bringup/rviz/lidar_mapping.rviz"
DEFAULT_SYSTEM_RVIZ="${ROS_WS}/src/robot_bringup/rviz/system.rviz"
DEFAULT_CAMERA_INFO_URL="file://${ROS_WS}/src/robot_bringup/config/camera_info/rgb_Astra_Orbbec.yaml"

TMP_FILES=()

cleanup() {
    local file
    for file in "${TMP_FILES[@]:-}"; do
        if [ -n "$file" ] && [ -f "$file" ]; then
            rm -f "$file"
        fi
    done
}

trap cleanup EXIT

register_tmp_file() {
    TMP_FILES+=("$1")
}

main_usage() {
    cat <<'EOF'
统一入口: ./robot.sh <command> [args]

命令:
  mapping       启动建图
  navigation    启动导航
  sensor        启动单一传感器
  base          启动 STM32 底盘桥接
  full          启动全量观测系统（雷达 + 相机 + 底盘 + 建图）
  check         执行健康检查（lidar 或 mapping）
  save-map      保存当前地图
  doctor        执行系统诊断
  teleop        启动键盘遥控
  stop          停止当前 ROS2 会话
  help          查看帮助

示例:
  ./robot.sh mapping lidar --real-base
  ./robot.sh navigation /home/robot/ros2_maps/my_map.yaml --real-base
  ./robot.sh sensor lidar
  ./robot.sh check lidar
  ./robot.sh doctor
EOF
}

mapping_usage() {
    cat <<'EOF'
用法: ./robot.sh mapping [camera|lidar] [auto|quality|precision|fast] [--skip-lidar-check] [--no-rviz] [--real-base|--fake-base] [--ekf-base] [--base-port PORT]
EOF
}

navigation_usage() {
    cat <<'EOF'
用法: ./robot.sh navigation [map.yaml] [--real-base|--fake-base] [--ekf-base] [--no-rviz] [--skip-lidar-check] [--base-port PORT]
EOF
}

check_usage() {
    cat <<'EOF'
用法:
  ./robot.sh check lidar [雷达参数文件]
  ./robot.sh check mapping [--allow-headless] [--min-scan-hz N]
EOF
}

resolve_base_port() {
    local port="$1"
    local detected

    if [ "$port" = "auto" ] && [ -x "$DETECT_BASE_PORT_SCRIPT" ]; then
        if [ -n "${ROBOT_BASE_PORT_HINT:-}" ]; then
            detected="${ROBOT_BASE_PORT_HINT}"
        else
            detected="$("$DETECT_BASE_PORT_SCRIPT" --probe 2>/dev/null || true)"
        fi
        if [ -n "$detected" ]; then
            printf '%s\n' "$detected"
            return 0
        fi
    fi

    printf '%s\n' "$port"
}

prepare_lidar_params() {
    local params_file="$1"
    local tmp_prefix="$2"
    local configured_port=""
    local detected_port=""
    local tmp_file=""

    configured_port="$(get_yaml_key_value "$params_file" "port" || true)"
    if [ -x "$DETECT_LIDAR_PORT_SCRIPT" ]; then
        detected_port="$("$DETECT_LIDAR_PORT_SCRIPT" "$configured_port" 2>/dev/null || true)"
    fi

    if [ -n "$detected_port" ] && [ "$detected_port" != "$configured_port" ]; then
        tmp_file="$(mktemp "/tmp/${tmp_prefix}_XXXX.yaml")"
        render_yaml_key_value "$params_file" "port" "$detected_port" "$tmp_file"
        register_tmp_file "$tmp_file"
        printf '%s\n' "$tmp_file"
        return 0
    fi

    printf '%s\n' "$params_file"
}

choose_slam_params() {
    local profile="$1"

    case "$profile" in
        quality)
            printf '%s\n' "$DEFAULT_SLAM_PARAMS"
            ;;
        precision)
            printf '%s\n' "$PRECISION_SLAM_PARAMS"
            ;;
        fast)
            printf '%s\n' "$FAST_SLAM_PARAMS"
            ;;
        *)
            log_error "✗ 未知 SLAM 参数档位: $profile"
            exit 1
            ;;
    esac
}

stop_existing_sessions() {
    if [ -x "$STOP_ALL_SCRIPT" ]; then
        log_info "清理旧 ROS2 会话..."
        "$STOP_ALL_SCRIPT" >/dev/null 2>&1 || true
        sleep 1
    fi
}

run_mapping() {
    local mapping_source="camera"
    local slam_profile="auto"
    local skip_lidar_check=false
    local use_rviz=true
    local base_mode="none"
    local base_port="${BASE_PORT:-auto}"
    local has_source=false
    local has_profile=false
    local slam_params=""
    local lidar_params=""
    local rviz_config="$DEFAULT_SYSTEM_RVIZ"
    local use_camera="false"
    local use_visual_odom="false"
    local use_base_arg="false"
    local base_imu_enabled="true"
    local base_fusion_mode="none"
    local lidar_port=""
    local arg=""

    while [ $# -gt 0 ]; do
        arg="$1"
        case "$arg" in
            camera|lidar)
                if [ "$has_source" = true ]; then
                    log_error "✗ 重复指定建图源: $arg"
                    mapping_usage
                    exit 1
                fi
                mapping_source="$arg"
                has_source=true
                shift
                ;;
            auto|quality|precision|fast)
                if [ "$has_profile" = true ]; then
                    log_error "✗ 重复指定参数档位: $arg"
                    mapping_usage
                    exit 1
                fi
                slam_profile="$arg"
                has_profile=true
                shift
                ;;
            --skip-lidar-check)
                skip_lidar_check=true
                shift
                ;;
            --no-rviz)
                use_rviz=false
                shift
                ;;
            --real-base)
                base_mode="real"
                use_base_arg="true"
                shift
                ;;
            --fake-base)
                base_mode="fake"
                use_base_arg="true"
                shift
                ;;
            --ekf-base)
                base_fusion_mode="ekf"
                shift
                ;;
            --base-port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --base-port 需要一个串口路径"
                    mapping_usage
                    exit 1
                fi
                base_port="$2"
                shift 2
                ;;
            -h|--help)
                mapping_usage
                exit 0
                ;;
            *)
                log_error "✗ 未知参数: $arg"
                mapping_usage
                exit 1
                ;;
        esac
    done

    ensure_workspace_built
    setup_ros_env
    print_header "建图模式启动"

    if [ "$slam_profile" = "auto" ]; then
        if [ "$mapping_source" = "lidar" ]; then
            slam_profile="quality"
        else
            slam_profile="fast"
        fi
    fi
    slam_params="$(choose_slam_params "$slam_profile")"

    if [ "$base_mode" = "real" ]; then
        base_port="$(resolve_base_port "$base_port")"
        if [ "$base_port" != "auto" ]; then
            export ROBOT_BASE_PORT_HINT="$base_port"
        fi
    fi

    if [ "$mapping_source" = "lidar" ]; then
        rviz_config="$DEFAULT_LIDAR_RVIZ"
        lidar_params="$DEFAULT_LIDAR_PARAMS"
        if [ ! -f "$lidar_params" ]; then
            lidar_params="$DEFAULT_LIDAR_FALLBACK_PARAMS"
        fi
        lidar_params="$(prepare_lidar_params "$lidar_params" "ydlidar_mapping")"
        lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
        if [ -z "$lidar_port" ] || [ ! -e "$lidar_port" ]; then
            log_error "✗ 未找到雷达设备"
            exit 1
        fi
        export ROBOT_LIDAR_PORT_HINT="$lidar_port"
        if [ "$skip_lidar_check" = false ] && [ -x "$LIDAR_HEALTH_SCRIPT" ]; then
            log_info "执行雷达数据健康检查..."
            "$LIDAR_HEALTH_SCRIPT" "$lidar_params"
        fi
    fi

    if [ "$mapping_source" = "camera" ]; then
        use_camera="true"
        if ros2 pkg list 2>/dev/null | grep -qx "rtabmap_odom"; then
            use_visual_odom="true"
        else
            log_warn "提示: 未检测到 rtabmap_odom，回退到无视觉里程计模式"
        fi
    fi

    if [ "$mapping_source" = "lidar" ] && [ "$base_mode" = "real" ]; then
        base_imu_enabled="false"
    fi

    log_info "系统组件:"
    if [ "$mapping_source" = "camera" ]; then
        echo "  • Astra Pro 深度相机 -> depthimage_to_laserscan"
    else
        echo "  • YDLIDAR X2 激光雷达"
    fi
    case "$base_mode" in
        real)
            echo "  • STM32 底盘里程计 (${base_port})"
            ;;
        fake)
            echo "  • 虚拟底盘里程计"
            ;;
        *)
            echo "  • 底盘里程计已禁用"
            ;;
    esac
    echo "  • SLAM Toolbox 建图引擎"
    if [ "$use_rviz" = true ]; then
        echo "  • RViz2 可视化界面"
    else
        echo "  • RViz2 已禁用"
    fi
    echo

    stop_existing_sessions

    log_success "正在启动统一建图系统..."
    log_info "入口: robot_bringup/system.launch.py"
    log_info "建图源: ${mapping_source}"
    log_info "参数档位: ${slam_profile}"
    log_info "SLAM 参数: ${slam_params}"
    if [ "$mapping_source" = "lidar" ]; then
        log_info "雷达参数: ${lidar_params}"
        log_info "雷达串口: ${lidar_port}"
    fi
    log_info "底盘模式: ${base_mode}"
    if [ "$base_mode" = "real" ]; then
        log_info "底盘串口: ${base_port}"
        log_info "底盘 IMU 参与控制: ${base_imu_enabled}"
        log_info "底盘融合模式: ${base_fusion_mode}"
    fi
    log_info "RViz 配置: ${rviz_config}"
    echo

    ros2 launch robot_bringup system.launch.py \
        mode:=mapping \
        mapping_source:=${mapping_source} \
        use_camera:=${use_camera} \
        base_mode:=${base_mode} \
        use_base:=${use_base_arg} \
        base_port:=${base_port} \
        base_imu_enabled:=${base_imu_enabled} \
        base_fusion_mode:=${base_fusion_mode} \
        use_visual_odom:=${use_visual_odom} \
        use_rviz:=${use_rviz} \
        lidar_params_file:=${lidar_params:-$DEFAULT_LIDAR_PARAMS} \
        rviz_config:=${rviz_config} \
        camera_use_uvc:=true \
        camera_enable_ir:=false \
        camera_color_info_url:=${DEFAULT_CAMERA_INFO_URL} \
        slam_params_file:=${slam_params}
}

run_navigation() {
    local map_file="$DEFAULT_MAP"
    local base_mode="real"
    local base_port="${BASE_PORT:-auto}"
    local base_fusion_mode="none"
    local use_rviz=true
    local skip_lidar_check=false
    local lidar_params=""
    local lidar_port=""
    local arg=""

    while [ $# -gt 0 ]; do
        arg="$1"
        case "$arg" in
            --fake-base)
                base_mode="fake"
                shift
                ;;
            --real-base)
                base_mode="real"
                shift
                ;;
            --ekf-base)
                base_fusion_mode="ekf"
                shift
                ;;
            --no-rviz)
                use_rviz=false
                shift
                ;;
            --skip-lidar-check)
                skip_lidar_check=true
                shift
                ;;
            --base-port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --base-port 需要一个串口路径"
                    navigation_usage
                    exit 1
                fi
                base_port="$2"
                shift 2
                ;;
            -h|--help)
                navigation_usage
                exit 0
                ;;
            *)
                if [[ "$arg" == --* ]]; then
                    log_error "✗ 未知参数: $arg"
                    navigation_usage
                    exit 1
                fi
                map_file="$arg"
                shift
                ;;
        esac
    done

    ensure_workspace_built
    setup_ros_env
    print_header "导航模式启动"

    if [ ! -f "$map_file" ]; then
        log_error "✗ 地图文件不存在: $map_file"
        exit 1
    fi

    if [ "$base_mode" = "real" ]; then
        base_port="$(resolve_base_port "$base_port")"
        if [ "$base_port" != "auto" ]; then
            export ROBOT_BASE_PORT_HINT="$base_port"
        fi
    fi

    lidar_params="$(prepare_lidar_params "$DEFAULT_LIDAR_PARAMS" "ydlidar_nav")"
    lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
    if [ -z "$lidar_port" ] || [ ! -e "$lidar_port" ]; then
        log_error "✗ 未找到雷达设备"
        exit 1
    fi
    export ROBOT_LIDAR_PORT_HINT="$lidar_port"

    if [ "$skip_lidar_check" = false ] && [ -x "$LIDAR_HEALTH_SCRIPT" ]; then
        log_info "执行雷达数据健康检查..."
        "$LIDAR_HEALTH_SCRIPT" "$lidar_params"
    fi

    stop_existing_sessions

    log_success "正在启动统一导航系统..."
    log_info "地图文件: ${map_file}"
    log_info "雷达串口: ${lidar_port}"
    log_info "底盘模式: ${base_mode}"
    if [ "$base_mode" = "real" ]; then
        log_info "底盘串口: ${base_port}"
        log_info "底盘融合模式: ${base_fusion_mode}"
        if [ "$base_port" = "auto" ]; then
            log_warn "⚠ 未检测到底盘串口，桥接节点将以 auto 模式持续重试"
        fi
    fi
    echo

    ros2 launch robot_bringup system.launch.py \
        mode:=navigation \
        use_camera:=false \
        use_lidar:=true \
        base_mode:=${base_mode} \
        base_fusion_mode:=${base_fusion_mode} \
        use_base:=true \
        use_rviz:=${use_rviz} \
        map_file:=${map_file} \
        base_port:=${base_port} \
        base_baudrate:=115200 \
        lidar_params_file:=${lidar_params}
}

run_sensor() {
    local sensor="${1:-}"
    local lidar_params=""
    local lidar_port=""

    if [ -z "$sensor" ]; then
        log_error "✗ 缺少传感器类型: lidar 或 camera"
        exit 1
    fi
    shift || true

    ensure_workspace_built
    setup_ros_env

    case "$sensor" in
        lidar)
            print_header "激光雷达传感器启动"
            lidar_params="$(prepare_lidar_params "$DEFAULT_LIDAR_FALLBACK_PARAMS" "ydlidar_sensor")"
            lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
            if [ -z "$lidar_port" ] || [ ! -e "$lidar_port" ]; then
                log_error "✗ 未找到雷达设备"
                exit 1
            fi
            log_info "雷达串口: ${lidar_port}"
            ros2 launch robot_bringup sensors.launch.py \
                use_lidar:=true \
                use_camera:=false \
                lidar_params_file:=${lidar_params}
            ;;
        camera)
            print_header "Astra Pro 摄像头启动"
            if [ ! -e /dev/video0 ] && [ ! -e /dev/video1 ]; then
                log_error "✗ 未找到摄像头设备"
                exit 1
            fi
            ros2 launch robot_bringup sensors.launch.py \
                use_lidar:=false \
                use_camera:=true
            ;;
        *)
            log_error "✗ 未知传感器类型: $sensor"
            exit 1
            ;;
    esac
}

run_base() {
    local base_port="${BASE_PORT:-auto}"

    while [ $# -gt 0 ]; do
        case "$1" in
            --base-port|--port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --port 需要一个串口路径"
                    exit 1
                fi
                base_port="$2"
                shift 2
                ;;
            -h|--help)
                echo "用法: ./robot.sh base [--port /dev/ttyUSB0]"
                exit 0
                ;;
            *)
                log_error "✗ 未知参数: $1"
                exit 1
                ;;
        esac
    done

    ensure_workspace_built
    setup_ros_env
    print_header "底盘桥接启动"
    base_port="$(resolve_base_port "$base_port")"
    if [ "$base_port" = "auto" ]; then
        log_warn "⚠ 暂未检测到底盘串口，桥接节点将以 auto 模式持续重试"
    else
        log_info "底盘串口: ${base_port}"
    fi
    ros2 launch stm32_robot_bridge stm32_bridge.launch.py \
        port:=${base_port} \
        baudrate:=115200
}

run_system() {
    local base_port="${BASE_PORT:-auto}"
    local use_rviz=true
    local skip_lidar_check=false
    local lidar_params=""
    local lidar_port=""

    while [ $# -gt 0 ]; do
        case "$1" in
            --base-port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --base-port 需要一个串口路径"
                    exit 1
                fi
                base_port="$2"
                shift 2
                ;;
            --skip-lidar-check)
                skip_lidar_check=true
                shift
                ;;
            --no-rviz)
                use_rviz=false
                shift
                ;;
            -h|--help)
                echo "用法: ./robot.sh system [--base-port PORT] [--skip-lidar-check] [--no-rviz]"
                exit 0
                ;;
            *)
                log_error "✗ 未知参数: $1"
                exit 1
                ;;
        esac
    done

    ensure_workspace_built
    setup_ros_env
    print_header "完整系统启动"

    base_port="$(resolve_base_port "$base_port")"
    lidar_params="$(prepare_lidar_params "$DEFAULT_LIDAR_PARAMS" "ydlidar_system")"
    lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
    if [ -z "$lidar_port" ] || [ ! -e "$lidar_port" ]; then
        log_error "✗ 未找到雷达设备"
        exit 1
    fi
    if [ "$skip_lidar_check" = false ] && [ -x "$LIDAR_HEALTH_SCRIPT" ]; then
        log_info "执行雷达数据健康检查..."
        "$LIDAR_HEALTH_SCRIPT" "$lidar_params"
    fi

    stop_existing_sessions

    log_info "系统组件:"
    echo "  • YDLIDAR X2 激光雷达 (${lidar_port})"
    echo "  • Astra Pro 深度相机"
    echo "  • STM32 串口桥接 (${base_port})"
    echo "  • SLAM Toolbox"
    if [ "$use_rviz" = true ]; then
        echo "  • RViz2"
    fi
    echo

    ros2 launch robot_bringup system.launch.py \
        mode:=mapping \
        mapping_source:=lidar \
        use_lidar:=true \
        use_camera:=true \
        base_mode:=real \
        use_base:=true \
        use_rviz:=${use_rviz} \
        base_port:=${base_port} \
        base_baudrate:=115200 \
        lidar_params_file:=${lidar_params}
}

run_passthrough_script() {
    local script_path="$1"
    shift
    exec "$script_path" "$@"
}

run_check() {
    local target="${1:-}"
    shift || true

    case "$target" in
        lidar)
            run_passthrough_script "$LIDAR_HEALTH_SCRIPT" "$@"
            ;;
        mapping)
            run_passthrough_script "$CHECK_MAPPING_SCRIPT" "$@"
            ;;
        -h|--help|"")
            check_usage
            exit 0
            ;;
        *)
            log_error "✗ 未知检查目标: ${target}"
            check_usage
            exit 1
            ;;
    esac
}

COMMAND="${1:-help}"
if [ $# -gt 0 ]; then
    shift
fi

case "$COMMAND" in
    mapping)
        run_mapping "$@"
        ;;
    navigation|nav)
        run_navigation "$@"
        ;;
    sensor|sensors)
        run_sensor "$@"
        ;;
    base|bridge)
        run_base "$@"
        ;;
    full|system)
        run_system "$@"
        ;;
    check)
        run_check "$@"
        ;;
    save-map)
        run_passthrough_script "$SAVE_MAP_SCRIPT" "$@"
        ;;
    check-lidar)
        run_passthrough_script "$LIDAR_HEALTH_SCRIPT" "$@"
        ;;
    doctor|check-system)
        run_passthrough_script "$CHECK_SYSTEM_SCRIPT" "$@"
        ;;
    teleop)
        run_passthrough_script "$KEYBOARD_CONTROL_SCRIPT" "$@"
        ;;
    stop)
        run_passthrough_script "$STOP_ALL_SCRIPT" "$@"
        ;;
    help|-h|--help)
        main_usage
        ;;
    *)
        log_error "✗ 未知命令: ${COMMAND}"
        main_usage
        exit 1
        ;;
esac
