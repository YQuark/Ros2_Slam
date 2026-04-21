#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="/home/robot/ros2_ws"

# shellcheck source=/home/robot/ros2_ws/launch_scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

LIDAR_HEALTH_SCRIPT="${SCRIPT_DIR}/check_lidar_health.sh"
DETECT_BASE_PORT_SCRIPT="${SCRIPT_DIR}/detect_base_port.sh"
STOP_ALL_SCRIPT="${SCRIPT_DIR}/stop_all.sh"
SAVE_MAP_SCRIPT="${SCRIPT_DIR}/save_map.sh"
CHECK_SYSTEM_SCRIPT="${SCRIPT_DIR}/check_system.sh"
CHECK_MAPPING_SCRIPT="${SCRIPT_DIR}/check_mapping_pipeline.sh"
KEYBOARD_CONTROL_SCRIPT="${SCRIPT_DIR}/keyboard_control.sh"
BASE_TEST_SCRIPT="${SCRIPT_DIR}/test_base_cmd.sh"

DEFAULT_MAP="/home/robot/ros2_maps/latest.yaml"
DEFAULT_SLAM_PARAMS="${ROS_WS}/src/robot_bringup/config/slam_toolbox_mapping.yaml"
PRECISION_SLAM_PARAMS="${ROS_WS}/src/robot_bringup/config/slam_toolbox_mapping_lidar_precision.yaml"
FAST_SLAM_PARAMS="${ROS_WS}/src/robot_bringup/config/slam_toolbox_mapping_fast.yaml"
DEFAULT_NAV2_MAPPING_PARAMS="${ROS_WS}/src/robot_bringup/config/nav2_mapping_params.yaml"
DEFAULT_LIDAR_PARAMS="${ROS_WS}/src/robot_bringup/config/ydlidar_X2_mapping.yaml"
DEFAULT_LIDAR_FALLBACK_PARAMS="${ROS_WS}/src/ydlidar_ros2_driver/params/X2.yaml"
DEFAULT_LIDAR_RVIZ="${ROS_WS}/src/robot_bringup/rviz/lidar_mapping.rviz"
DEFAULT_SYSTEM_RVIZ="${ROS_WS}/src/robot_bringup/rviz/system.rviz"
DEFAULT_CAMERA_INFO_URL="file://${ROS_WS}/src/robot_bringup/config/camera_info/rgb_Astra_Orbbec.yaml"
DEFAULT_LIDAR_TF_YAW_RAD="-1.570796326795"

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
  base-test     只测试上位机 /cmd_vel 控制底盘
  full          启动全量观测系统（雷达 + 相机 + 底盘 + 建图）
  check         执行健康检查（lidar 或 mapping）
  save-map      保存当前地图
  doctor        执行系统诊断
  teleop        启动键盘遥控
  stop          停止当前 ROS2 会话
  help          查看帮助

示例:
  ./robot.sh mapping lidar --real-base
  ./robot.sh save-map my_map
  ./robot.sh navigation --real-base --ekf-base
  ./robot.sh navigation /home/robot/ros2_maps/my_map.yaml --real-base
  ./robot.sh base-test --rotate-only
  ./robot.sh sensor lidar
  ./robot.sh check lidar
  ./robot.sh doctor
EOF
}

mapping_usage() {
    cat <<'EOF'
用法: ./robot.sh mapping [camera|lidar] [auto|quality|precision|fast] [--skip-lidar-check] [--no-rviz] [--real-base|--fake-base] [--ekf-base] [--base-port PORT] [--lidar-yaw-rad RAD|--lidar-yaw-deg DEG] [--lidar-reversion|--no-lidar-reversion] [--lidar-inverted|--no-lidar-inverted] [--auto-drive] [--auto-drive-duration SEC]

说明:
  --auto-drive 使用 Nav2 + frontier_explorer 自动选择未知边界目标，不再直接发布 /cmd_vel。
  可用 AUTO_MAPPING_GOAL_CLEARANCE_RADIUS / AUTO_MAPPING_GOAL_UNKNOWN_CLEARANCE_RADIUS 临时放宽或收紧贴墙距离。
  lidar 源建图时可用 --lidar-reversion / --lidar-inverted / --lidar-yaw-* 做运行时覆盖。
EOF
}

navigation_usage() {
    cat <<'EOF'
用法: ./robot.sh navigation [map.yaml] [--real-base|--fake-base] [--ekf-base] [--base-use-status-yaw|--no-base-status-yaw] [--base-odom-source status_twist|wheel_cps] [--lidar-yaw-rad RAD|--lidar-yaw-deg DEG] [--lidar-reversion|--no-lidar-reversion] [--lidar-inverted|--no-lidar-inverted] [--no-rviz] [--skip-lidar-check] [--base-port PORT] [--localization-only|--nav2-only]

说明:
  map.yaml 省略时默认使用 /home/robot/ros2_maps/latest.yaml。
  RViz 中先用 2D Pose Estimate 设置当前位置，再用 2D Goal Pose 点目标导航。
  --localization-only 只启动雷达、底盘、AMCL 和 RViz，先完成初始定位。
  --nav2-only         在初始定位完成后，单独启动 Nav2 规划和控制节点。
  若 2D Pose Estimate 箭头与点云朝向不一致，优先测试 --lidar-reversion / --lidar-inverted，再考虑 lidar_tf_yaw。
  若运动时 scan 不贴 map，优先测试 --no-base-status-yaw 或 --base-odom-source wheel_cps。
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

resolve_base_port_avoiding_lidar() {
    local mode_label="$1"
    local requested_base_port="$2"
    local lidar_port="$3"
    local base_port_explicit="$4"
    local resolved_base=""
    local resolved_lidar=""
    local resolved_base_port=""

    if [ -z "$lidar_port" ]; then
        resolve_base_port "$requested_base_port"
        return 0
    fi

    resolved_lidar="$(resolve_port "$lidar_port")"

    if [ "$requested_base_port" != "auto" ]; then
        resolved_base="$(resolve_port "$requested_base_port")"
        if [ "$resolved_base" = "$resolved_lidar" ]; then
            if [ "$base_port_explicit" = true ]; then
                log_error "✗ 底盘串口和雷达串口相同: ${requested_base_port}"
                log_error "  雷达已经占用 ${lidar_port}，请用 --base-port 指定另一个 STM32 串口，或交换 USB 后重试。"
                exit 1
            fi
            log_warn "检测到默认底盘串口 ${requested_base_port} 与雷达串口 ${lidar_port} 冲突，改为主动探测底盘并排除雷达口" >&2
            requested_base_port="auto"
        fi
    fi

    resolved_base_port="$(resolve_base_port "$requested_base_port")"
    if [ "$resolved_base_port" = "auto" ]; then
        log_error "✗ 未能探测到可用的 STM32 底盘串口"
        log_error "  ${mode_label} 已确认雷达占用 ${lidar_port}，不能让底盘继续以 auto 模式冒险抢占雷达口。"
        log_error "  请确认 STM32 已连接并能响应 GET_STATUS，或用 --base-port 指定非雷达串口。"
        exit 1
    fi
    if [ ! -e "$resolved_base_port" ]; then
        log_error "✗ 底盘串口不存在: ${resolved_base_port}"
        log_error "  请确认 STM32 已接入并能响应 GET_STATUS，或用 --base-port 指定实际底盘串口。"
        exit 1
    fi

    resolved_base="$(resolve_port "$resolved_base_port")"
    if [ "$resolved_base" = "$resolved_lidar" ]; then
        log_error "✗ 底盘串口和雷达串口仍然相同: ${resolved_base_port}"
        log_error "  请确认 STM32 已连接，并不要让底盘和雷达共用同一个 /dev/ttyUSB*。"
        exit 1
    fi

    printf '%s\n' "$resolved_base_port"
}

prepare_lidar_params() {
    local params_file="$1"
    local configured_port=""

    configured_port="$(get_yaml_key_value "$params_file" "port" || true)"
    if [ -z "$configured_port" ]; then
        log_error "✗ 雷达参数文件缺少 port: ${params_file}"
        exit 1
    fi

    printf '%s\n' "$params_file"
}

prepare_lidar_params_with_runtime_overrides() {
    local params_file="$1"
    local reversion_override="${2:-}"
    local inverted_override="${3:-}"
    local tmp_file=""

    if [ -z "$reversion_override" ] && [ -z "$inverted_override" ]; then
        printf '%s\n' "$params_file"
        return 0
    fi

    tmp_file="$(mktemp /tmp/robot_lidar_params.XXXXXX.yaml)"
    cp "$params_file" "$tmp_file"
    register_tmp_file "$tmp_file"

    if [ -n "$reversion_override" ]; then
        render_yaml_key_value "$tmp_file" "reversion" "$reversion_override" "$tmp_file.tmp"
        mv "$tmp_file.tmp" "$tmp_file"
    fi
    if [ -n "$inverted_override" ]; then
        render_yaml_key_value "$tmp_file" "inverted" "$inverted_override" "$tmp_file.tmp"
        mv "$tmp_file.tmp" "$tmp_file"
    fi

    printf '%s\n' "$tmp_file"
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

deg_to_rad() {
    awk -v deg="$1" 'BEGIN { printf "%.12f\n", deg * atan2(0, -1) / 180.0 }'
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
    local base_port="${BASE_PORT:-/dev/ttyUSB1}"
    local has_source=false
    local has_profile=false
    local slam_params=""
    local lidar_params=""
    local rviz_config="$DEFAULT_SYSTEM_RVIZ"
    local use_camera="false"
    local use_visual_odom="false"
    local use_base_arg="false"
    local base_imu_enabled="${BASE_IMU_ENABLED:-true}"
    local base_fusion_mode="none"
    local base_use_status_yaw="${BASE_USE_STATUS_YAW:-true}"
    local base_status_yaw_mode="${BASE_STATUS_YAW_MODE:-relative}"
    local base_status_yaw_jump_reject_deg="${BASE_STATUS_YAW_JUMP_REJECT_DEG:-25.0}"
    local base_max_linear="${BASE_MAX_LINEAR:-1.20}"
    local base_max_angular="${BASE_MAX_ANGULAR:-19.27}"
    local base_odom_feedback_source="${BASE_ODOM_FEEDBACK_SOURCE:-status_twist}"
    local base_wheel_track_width="${BASE_WHEEL_TRACK_WIDTH:-0.1250}"
    local base_odom_angular_scale="${BASE_ODOM_ANGULAR_SCALE:-1.0}"
    local base_odom_angular_sign="${BASE_ODOM_ANGULAR_SIGN:-1.0}"
    local base_status_log_interval_sec="${BASE_STATUS_LOG_INTERVAL_SEC:-0.0}"
    local base_cmd_log_interval_sec="${BASE_CMD_LOG_INTERVAL_SEC:-0.0}"
    local auto_mapping_drive="false"
    local auto_mapping_nav2_params="${AUTO_MAPPING_NAV2_PARAMS:-$DEFAULT_NAV2_MAPPING_PARAMS}"
    local auto_mapping_max_duration_sec="${AUTO_MAPPING_MAX_DURATION_SEC:-180.0}"
    local auto_mapping_frontier_min_distance="${AUTO_MAPPING_FRONTIER_MIN_DISTANCE:-0.70}"
    local auto_mapping_frontier_max_distance="${AUTO_MAPPING_FRONTIER_MAX_DISTANCE:-2.5}"
    local auto_mapping_min_frontier_cluster_cells="${AUTO_MAPPING_MIN_FRONTIER_CLUSTER_CELLS:-10}"
    local auto_mapping_goal_approach_offset="${AUTO_MAPPING_GOAL_APPROACH_OFFSET:-0.55}"
    local auto_mapping_goal_approach_max_offset="${AUTO_MAPPING_GOAL_APPROACH_MAX_OFFSET:-0.90}"
    local auto_mapping_goal_approach_step="${AUTO_MAPPING_GOAL_APPROACH_STEP:-0.05}"
    local auto_mapping_goal_clearance_radius="${AUTO_MAPPING_GOAL_CLEARANCE_RADIUS:-0.38}"
    local auto_mapping_goal_unknown_clearance_radius="${AUTO_MAPPING_GOAL_UNKNOWN_CLEARANCE_RADIUS:-0.35}"
    local auto_mapping_linear_speed="${AUTO_MAPPING_LINEAR_SPEED:-0.10}"
    local auto_mapping_turn_speed="${AUTO_MAPPING_TURN_SPEED:-1.65}"
    local auto_mapping_emergency_stop_distance="${AUTO_MAPPING_EMERGENCY_STOP_DISTANCE:-0.25}"
    local auto_mapping_front_stop_distance="${AUTO_MAPPING_FRONT_STOP_DISTANCE:-0.40}"
    local auto_mapping_front_resume_distance="${AUTO_MAPPING_FRONT_RESUME_DISTANCE:-0.48}"
    local auto_mapping_front_slow_distance="${AUTO_MAPPING_FRONT_SLOW_DISTANCE:-0.80}"
    local auto_mapping_side_emergency_stop_distance="${AUTO_MAPPING_SIDE_EMERGENCY_STOP_DISTANCE:-0.12}"
    local auto_mapping_side_stop_distance="${AUTO_MAPPING_SIDE_STOP_DISTANCE:-0.10}"
    local auto_mapping_side_resume_distance="${AUTO_MAPPING_SIDE_RESUME_DISTANCE:-0.18}"
    local lidar_tf_yaw="${LIDAR_TF_YAW:-$DEFAULT_LIDAR_TF_YAW_RAD}"
    local lidar_reversion_override=""
    local lidar_inverted_override=""
    local lidar_port=""
    local base_port_explicit=false
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
                base_port_explicit=true
                shift 2
                ;;
            --lidar-yaw-rad)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-rad 需要一个弧度值"
                    mapping_usage
                    exit 1
                fi
                lidar_tf_yaw="$2"
                shift 2
                ;;
            --lidar-yaw-deg)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-deg 需要一个角度值"
                    mapping_usage
                    exit 1
                fi
                lidar_tf_yaw="$(deg_to_rad "$2")"
                shift 2
                ;;
            --lidar-reversion)
                lidar_reversion_override="true"
                shift
                ;;
            --no-lidar-reversion)
                lidar_reversion_override="false"
                shift
                ;;
            --lidar-inverted)
                lidar_inverted_override="true"
                shift
                ;;
            --no-lidar-inverted)
                lidar_inverted_override="false"
                shift
                ;;
            --auto-drive)
                auto_mapping_drive="true"
                shift
                ;;
            --auto-drive-duration)
                if [ $# -lt 2 ]; then
                    log_error "✗ --auto-drive-duration 需要秒数"
                    mapping_usage
                    exit 1
                fi
                auto_mapping_max_duration_sec="$2"
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

    if [ "$mapping_source" = "lidar" ]; then
        rviz_config="$DEFAULT_LIDAR_RVIZ"
        lidar_params="$DEFAULT_LIDAR_PARAMS"
        if [ ! -f "$lidar_params" ]; then
            lidar_params="$DEFAULT_LIDAR_FALLBACK_PARAMS"
        fi
        lidar_params="$(prepare_lidar_params "$lidar_params" "ydlidar_mapping")"
        lidar_params="$(prepare_lidar_params_with_runtime_overrides "$lidar_params" "$lidar_reversion_override" "$lidar_inverted_override")"
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

    if [ "$base_mode" = "real" ]; then
        base_port="$(resolve_base_port_avoiding_lidar "建图模式" "$base_port" "$lidar_port" "$base_port_explicit")"
        export ROBOT_BASE_PORT_HINT="$base_port"
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
        base_imu_enabled="${BASE_IMU_ENABLED:-true}"
        base_use_status_yaw="${BASE_USE_STATUS_YAW:-true}"
        base_max_linear="${BASE_MAX_LINEAR:-1.20}"
        base_max_angular="${BASE_MAX_ANGULAR:-19.27}"
        base_odom_feedback_source="${BASE_ODOM_FEEDBACK_SOURCE:-status_twist}"
        base_odom_angular_scale="${BASE_ODOM_ANGULAR_SCALE:-1.0}"
        if [ "${BASE_STATUS_LOG_INTERVAL_SEC:-}" = "" ]; then
            base_status_log_interval_sec="5.0"
        fi
    fi

    if [ "$auto_mapping_drive" = "true" ] && [ "${BASE_STATUS_LOG_INTERVAL_SEC:-}" = "" ]; then
        base_status_log_interval_sec="10.0"
    fi
    if [ "$auto_mapping_drive" = "true" ] && [ "${BASE_CMD_LOG_INTERVAL_SEC:-}" = "" ]; then
        base_cmd_log_interval_sec="5.0"
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
        log_info "雷达 yaw 外参(rad): ${lidar_tf_yaw}"
        if [ -n "$lidar_reversion_override" ]; then
            log_info "雷达 reversion 覆写: ${lidar_reversion_override}"
        fi
        if [ -n "$lidar_inverted_override" ]; then
            log_info "雷达 inverted 覆写: ${lidar_inverted_override}"
        fi
    fi
    log_info "底盘模式: ${base_mode}"
    if [ "$base_mode" = "real" ]; then
        log_info "底盘串口: ${base_port}"
        log_info "底盘 IMU 参与控制: ${base_imu_enabled}"
        log_info "底盘融合模式: ${base_fusion_mode}"
    fi
    if [ "$auto_mapping_drive" = "true" ]; then
        log_warn "自动建图巡航已启用: 请保持人工急停可用，不要让 PS2/ESP 同时抢控制"
        log_info "自动探索: Nav2 frontier, ${auto_mapping_max_duration_sec}s"
        log_info "frontier 安全点: dist=${auto_mapping_frontier_min_distance}-${auto_mapping_frontier_max_distance}m approach=${auto_mapping_goal_approach_offset}-${auto_mapping_goal_approach_max_offset}m clear=${auto_mapping_goal_clearance_radius}m unknown_clear=${auto_mapping_goal_unknown_clearance_radius}m"
        log_info "Nav2 建图参数: ${auto_mapping_nav2_params}"
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
        base_max_linear:=${base_max_linear} \
        base_max_angular:=${base_max_angular} \
        base_imu_enabled:=${base_imu_enabled} \
        base_fusion_mode:=${base_fusion_mode} \
        base_use_status_yaw:=${base_use_status_yaw} \
        base_status_yaw_mode:=${base_status_yaw_mode} \
        base_status_yaw_jump_reject_deg:=${base_status_yaw_jump_reject_deg} \
        base_odom_feedback_source:=${base_odom_feedback_source} \
        base_wheel_track_width:=${base_wheel_track_width} \
        base_odom_angular_scale:=${base_odom_angular_scale} \
        base_odom_angular_sign:=${base_odom_angular_sign} \
        base_status_log_interval_sec:=${base_status_log_interval_sec} \
        base_cmd_log_interval_sec:=${base_cmd_log_interval_sec} \
        auto_mapping_drive:=${auto_mapping_drive} \
        auto_mapping_nav2_params_file:=${auto_mapping_nav2_params} \
        auto_mapping_max_duration_sec:=${auto_mapping_max_duration_sec} \
        auto_mapping_frontier_min_distance:=${auto_mapping_frontier_min_distance} \
        auto_mapping_frontier_max_distance:=${auto_mapping_frontier_max_distance} \
        auto_mapping_min_frontier_cluster_cells:=${auto_mapping_min_frontier_cluster_cells} \
        auto_mapping_goal_approach_offset:=${auto_mapping_goal_approach_offset} \
        auto_mapping_goal_approach_max_offset:=${auto_mapping_goal_approach_max_offset} \
        auto_mapping_goal_approach_step:=${auto_mapping_goal_approach_step} \
        auto_mapping_goal_clearance_radius:=${auto_mapping_goal_clearance_radius} \
        auto_mapping_goal_unknown_clearance_radius:=${auto_mapping_goal_unknown_clearance_radius} \
        auto_mapping_linear_speed:=${auto_mapping_linear_speed} \
        auto_mapping_turn_speed:=${auto_mapping_turn_speed} \
        auto_mapping_emergency_stop_distance:=${auto_mapping_emergency_stop_distance} \
        auto_mapping_front_stop_distance:=${auto_mapping_front_stop_distance} \
        auto_mapping_front_resume_distance:=${auto_mapping_front_resume_distance} \
        auto_mapping_front_slow_distance:=${auto_mapping_front_slow_distance} \
        auto_mapping_side_emergency_stop_distance:=${auto_mapping_side_emergency_stop_distance} \
        auto_mapping_side_stop_distance:=${auto_mapping_side_stop_distance} \
        auto_mapping_side_resume_distance:=${auto_mapping_side_resume_distance} \
        use_visual_odom:=${use_visual_odom} \
        use_rviz:=${use_rviz} \
        lidar_tf_yaw:=${lidar_tf_yaw} \
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
    local base_port="${BASE_PORT:-/dev/ttyUSB1}"
    local base_imu_enabled="true"
    local base_fusion_mode="none"
    local base_use_status_yaw="${BASE_USE_STATUS_YAW:-true}"
    local base_status_yaw_mode="${BASE_STATUS_YAW_MODE:-relative}"
    local base_status_yaw_jump_reject_deg="${BASE_STATUS_YAW_JUMP_REJECT_DEG:-25.0}"
    local base_odom_feedback_source="${BASE_ODOM_FEEDBACK_SOURCE:-status_twist}"
    local base_wheel_track_width="${BASE_WHEEL_TRACK_WIDTH:-0.1250}"
    local base_odom_angular_scale="${BASE_ODOM_ANGULAR_SCALE:-1.0}"
    local base_odom_angular_sign="${BASE_ODOM_ANGULAR_SIGN:-1.0}"
    local base_status_log_interval_sec="${BASE_STATUS_LOG_INTERVAL_SEC:-1.0}"
    local base_cmd_log_interval_sec="${BASE_CMD_LOG_INTERVAL_SEC:-1.0}"
    local base_cmd_timeout="${BASE_CMD_TIMEOUT:-1.0}"
    local base_drive_keepalive_sec="${BASE_DRIVE_KEEPALIVE_SEC:-0.10}"
    local use_rviz=true
    local skip_lidar_check=false
    local lidar_params=""
    local lidar_port=""
    local lidar_tf_yaw="${LIDAR_TF_YAW:-$DEFAULT_LIDAR_TF_YAW_RAD}"
    local lidar_reversion_override=""
    local lidar_inverted_override=""
    local base_port_explicit=false
    local localization_only=false
    local nav2_only=false
    local nav2_params_file="${ROS_WS}/src/robot_bringup/config/nav2_params_robot.yaml"
    local nav2_bt_xml_file="${ROS_WS}/src/robot_bringup/behavior_trees/mapping_navigate_to_pose.xml"
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
            --base-use-status-yaw)
                base_use_status_yaw="true"
                shift
                ;;
            --no-base-status-yaw)
                base_use_status_yaw="false"
                shift
                ;;
            --base-odom-source)
                if [ $# -lt 2 ]; then
                    log_error "✗ --base-odom-source 需要 status_twist 或 wheel_cps"
                    navigation_usage
                    exit 1
                fi
                case "$2" in
                    status_twist|wheel_cps)
                        base_odom_feedback_source="$2"
                        ;;
                    *)
                        log_error "✗ 无效的 --base-odom-source: $2"
                        navigation_usage
                        exit 1
                        ;;
                esac
                shift 2
                ;;
            --no-rviz)
                use_rviz=false
                shift
                ;;
            --skip-lidar-check)
                skip_lidar_check=true
                shift
                ;;
            --localization-only)
                localization_only=true
                shift
                ;;
            --nav2-only)
                nav2_only=true
                shift
                ;;
            --base-port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --base-port 需要一个串口路径"
                    navigation_usage
                    exit 1
                fi
                base_port="$2"
                base_port_explicit=true
                shift 2
                ;;
            --lidar-yaw-rad)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-rad 需要一个弧度值"
                    navigation_usage
                    exit 1
                fi
                lidar_tf_yaw="$2"
                shift 2
                ;;
            --lidar-yaw-deg)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-deg 需要一个角度值"
                    navigation_usage
                    exit 1
                fi
                lidar_tf_yaw="$(deg_to_rad "$2")"
                shift 2
                ;;
            --lidar-reversion)
                lidar_reversion_override="true"
                shift
                ;;
            --no-lidar-reversion)
                lidar_reversion_override="false"
                shift
                ;;
            --lidar-inverted)
                lidar_inverted_override="true"
                shift
                ;;
            --no-lidar-inverted)
                lidar_inverted_override="false"
                shift
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
    if [ "$localization_only" = true ] && [ "$nav2_only" = true ]; then
        log_error "✗ --localization-only 和 --nav2-only 不能同时使用"
        navigation_usage
        exit 1
    fi

    if [ "$nav2_only" = true ]; then
        print_header "Nav2 规划控制启动"
        log_info "前提: 已经另一个终端运行 ./robot.sh navigation --localization-only"
        log_info "前提: RViz 已用 2D Pose Estimate 完成初始定位，并且 map->base_link TF 可用"
        log_info "Nav2 参数: ${nav2_params_file}"
        echo
        ros2 launch robot_bringup nav2.launch.py \
            use_sim_time:=false \
            params_file:=${nav2_params_file} \
            default_bt_xml_filename:=${nav2_bt_xml_file} \
            autostart:=true
        return
    fi

    print_header "导航模式启动"

    if [ ! -f "$map_file" ]; then
        log_error "✗ 地图文件不存在: $map_file"
        exit 1
    fi

    lidar_params="$(prepare_lidar_params "$DEFAULT_LIDAR_PARAMS" "ydlidar_nav")"
    lidar_params="$(prepare_lidar_params_with_runtime_overrides "$lidar_params" "$lidar_reversion_override" "$lidar_inverted_override")"
    lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
    if [ -z "$lidar_port" ] || [ ! -e "$lidar_port" ]; then
        log_error "✗ 未找到雷达设备"
        exit 1
    fi
    export ROBOT_LIDAR_PORT_HINT="$lidar_port"

    if [ "$base_mode" = "real" ]; then
        base_port="$(resolve_base_port_avoiding_lidar "导航模式" "$base_port" "$lidar_port" "$base_port_explicit")"
        export ROBOT_BASE_PORT_HINT="$base_port"
    fi

    if [ "$skip_lidar_check" = false ] && [ -x "$LIDAR_HEALTH_SCRIPT" ]; then
        log_info "执行雷达数据健康检查..."
        "$LIDAR_HEALTH_SCRIPT" "$lidar_params"
    fi

    stop_existing_sessions

    if [ "$localization_only" = true ]; then
        log_success "正在启动定位阶段（雷达 + 底盘 + AMCL + RViz，不启动 Nav2）..."
    else
        log_success "正在启动统一导航系统..."
    fi
    local odom_source_label="bridge odom"
    if [ "$base_fusion_mode" = "ekf" ]; then
        odom_source_label="ekf odom"
    fi
    log_info "地图文件: ${map_file}"
    log_info "雷达参数: ${lidar_params}"
    log_info "雷达串口: ${lidar_port}"
    log_info "雷达 yaw 外参(rad): ${lidar_tf_yaw}"
    if [ -n "$lidar_reversion_override" ]; then
        log_info "雷达 reversion 覆写: ${lidar_reversion_override}"
    fi
    if [ -n "$lidar_inverted_override" ]; then
        log_info "雷达 inverted 覆写: ${lidar_inverted_override}"
    fi
    log_info "底盘模式: ${base_mode}"
    log_info "导航 odom 来源: ${odom_source_label}"
    if [ "$base_mode" = "real" ]; then
        log_info "底盘串口: ${base_port}"
        log_info "底盘 IMU 参与控制: ${base_imu_enabled}"
        log_info "底盘融合模式: ${base_fusion_mode}"
        log_info "底盘航向来源: $( [ "$base_use_status_yaw" = "true" ] && printf '下位机 yaw_est' || printf '桥接积分 w_est' )"
        log_info "底盘速度来源: ${base_odom_feedback_source}"
        log_info "底盘命令保活: timeout=${base_cmd_timeout}s keepalive=${base_drive_keepalive_sec}s"
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
        base_imu_enabled:=${base_imu_enabled} \
        base_use_status_yaw:=${base_use_status_yaw} \
        base_status_yaw_mode:=${base_status_yaw_mode} \
        base_status_yaw_jump_reject_deg:=${base_status_yaw_jump_reject_deg} \
        base_odom_feedback_source:=${base_odom_feedback_source} \
        base_wheel_track_width:=${base_wheel_track_width} \
        base_odom_angular_scale:=${base_odom_angular_scale} \
        base_odom_angular_sign:=${base_odom_angular_sign} \
        base_status_log_interval_sec:=${base_status_log_interval_sec} \
        base_cmd_log_interval_sec:=${base_cmd_log_interval_sec} \
        base_cmd_timeout:=${base_cmd_timeout} \
        base_drive_keepalive_sec:=${base_drive_keepalive_sec} \
        lidar_tf_yaw:=${lidar_tf_yaw} \
        lidar_params_file:=${lidar_params} \
        nav2_start:=$( [ "$localization_only" = true ] && printf 'false' || printf 'true' )
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
            lidar_params="$DEFAULT_LIDAR_PARAMS"
            if [ ! -f "$lidar_params" ]; then
                lidar_params="$DEFAULT_LIDAR_FALLBACK_PARAMS"
            fi
            lidar_params="$(prepare_lidar_params "$lidar_params" "ydlidar_sensor")"
            lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
            if [ -z "$lidar_port" ] || [ ! -e "$lidar_port" ]; then
                log_error "✗ 未找到雷达设备"
                exit 1
            fi
            log_info "雷达串口: ${lidar_port}"
            log_info "雷达参数: ${lidar_params}"
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
    local base_port="${BASE_PORT:-/dev/ttyUSB1}"
    local use_status_yaw="${BASE_USE_STATUS_YAW:-true}"
    local status_yaw_mode="${BASE_STATUS_YAW_MODE:-relative}"
    local status_yaw_jump_reject_deg="${BASE_STATUS_YAW_JUMP_REJECT_DEG:-25.0}"
    local base_wheel_track_width="${BASE_WHEEL_TRACK_WIDTH:-0.1250}"
    local base_odom_angular_scale="${BASE_ODOM_ANGULAR_SCALE:-1.0}"
    local base_odom_angular_sign="${BASE_ODOM_ANGULAR_SIGN:-1.0}"
    local base_status_log_interval_sec="${BASE_STATUS_LOG_INTERVAL_SEC:-0.0}"

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
            --use-status-yaw|--base-use-status-yaw)
                use_status_yaw="true"
                shift
                ;;
            -h|--help)
                echo "用法: ./robot.sh base [--port /dev/ttyUSB1] [--use-status-yaw]"
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
    log_info "底盘航向来源: $( [ "$use_status_yaw" = "true" ] && printf '下位机 yaw_est' || printf '桥接积分 w_est' )"
    ros2 launch stm32_robot_bridge stm32_bridge.launch.py \
        port:=${base_port} \
        baudrate:=115200 \
        use_status_yaw:=${use_status_yaw} \
        status_yaw_mode:=${status_yaw_mode} \
        status_yaw_jump_reject_deg:=${status_yaw_jump_reject_deg} \
        wheel_track_width:=${base_wheel_track_width} \
        odom_angular_scale:=${base_odom_angular_scale} \
        odom_angular_sign:=${base_odom_angular_sign} \
        status_log_interval_sec:=${base_status_log_interval_sec}
}

run_system() {
    local base_port="${BASE_PORT:-/dev/ttyUSB1}"
    local use_rviz=true
    local skip_lidar_check=false
    local lidar_params=""
    local lidar_port=""
    local lidar_tf_yaw="${LIDAR_TF_YAW:-$DEFAULT_LIDAR_TF_YAW_RAD}"
    local base_port_explicit=false
    local base_wheel_track_width="${BASE_WHEEL_TRACK_WIDTH:-0.1250}"
    local base_odom_angular_scale="${BASE_ODOM_ANGULAR_SCALE:-1.0}"
    local base_odom_angular_sign="${BASE_ODOM_ANGULAR_SIGN:-1.0}"
    local base_status_yaw_mode="${BASE_STATUS_YAW_MODE:-relative}"
    local base_status_yaw_jump_reject_deg="${BASE_STATUS_YAW_JUMP_REJECT_DEG:-25.0}"
    local base_status_log_interval_sec="${BASE_STATUS_LOG_INTERVAL_SEC:-0.0}"

    while [ $# -gt 0 ]; do
        case "$1" in
            --base-port)
                if [ $# -lt 2 ]; then
                    log_error "✗ --base-port 需要一个串口路径"
                    exit 1
                fi
                base_port="$2"
                base_port_explicit=true
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
            --lidar-yaw-rad)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-rad 需要一个弧度值"
                    exit 1
                fi
                lidar_tf_yaw="$2"
                shift 2
                ;;
            --lidar-yaw-deg)
                if [ $# -lt 2 ]; then
                    log_error "✗ --lidar-yaw-deg 需要一个角度值"
                    exit 1
                fi
                lidar_tf_yaw="$(deg_to_rad "$2")"
                shift 2
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

    lidar_params="$(prepare_lidar_params "$DEFAULT_LIDAR_PARAMS" "ydlidar_system")"
    lidar_port="$(get_yaml_key_value "$lidar_params" "port" || true)"
    if [ -z "$lidar_port" ] || [ ! -e "$lidar_port" ]; then
        log_error "✗ 未找到雷达设备"
        exit 1
    fi
    export ROBOT_LIDAR_PORT_HINT="$lidar_port"
    base_port="$(resolve_base_port_avoiding_lidar "完整系统" "$base_port" "$lidar_port" "$base_port_explicit")"
    export ROBOT_BASE_PORT_HINT="$base_port"
    if [ "$skip_lidar_check" = false ] && [ -x "$LIDAR_HEALTH_SCRIPT" ]; then
        log_info "执行雷达数据健康检查..."
        "$LIDAR_HEALTH_SCRIPT" "$lidar_params"
    fi

    stop_existing_sessions

    log_info "系统组件:"
    echo "  • YDLIDAR X2 激光雷达 (${lidar_port})"
    echo "  • 雷达 yaw 外参(rad): ${lidar_tf_yaw}"
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
        base_wheel_track_width:=${base_wheel_track_width} \
        base_odom_angular_scale:=${base_odom_angular_scale} \
        base_odom_angular_sign:=${base_odom_angular_sign} \
        base_status_yaw_mode:=${base_status_yaw_mode} \
        base_status_yaw_jump_reject_deg:=${base_status_yaw_jump_reject_deg} \
        base_status_log_interval_sec:=${base_status_log_interval_sec} \
        lidar_tf_yaw:=${lidar_tf_yaw} \
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
    base-test|test-base)
        run_passthrough_script "$BASE_TEST_SCRIPT" "$@"
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
