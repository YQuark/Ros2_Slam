#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=/home/robot/ros2_ws/launch_scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

sample_duration="${LOCALIZATION_CHECK_SAMPLE_SEC:-5}"
tf_sample_timeout="${LOCALIZATION_CHECK_TF_TIMEOUT_SEC:-5}"

print_notes() {
    cat <<'EOF'

请在测试过程中记录:
  1. odom yaw 方向是否正确
  2. odom yaw 角度是否接近真实角度
  3. scan 是否始终贴着 map

建议测试动作:
  A. 原地逆时针转 90°
  B. 直行 1m

症状对照:
  - 如果 2D Pose Estimate 箭头朝前，但点云整体方向不对，先检查雷达驱动 reversion/inverted，再检查 base_link -> laser_frame yaw。
  - 如果运动时 map 基本不动、scan 在来回摆，优先检查 odom -> base_link 的 yaw 方向、角度和连续性。
EOF
}

run_sample() {
    local title="$1"
    shift

    echo
    log_info "${title}"
    timeout "${sample_duration}s" "$@" || true
}

run_tf_sample() {
    local title="$1"
    shift

    echo
    log_info "${title}"
    timeout "${tf_sample_timeout}s" "$@" || true
}

main() {
    ensure_workspace_built
    setup_ros_env

    print_header "定位链路标定检查"
    log_info "采样时长: topic hz=${sample_duration}s, tf2_echo=${tf_sample_timeout}s"
    print_notes

    echo
    read -r -p "按 Enter 开始静态采样..." _

    run_sample "ros2 topic hz /odom" ros2 topic hz /odom
    run_sample "ros2 topic hz /scan" ros2 topic hz /scan
    run_sample "ros2 topic hz /particlecloud" ros2 topic hz /particlecloud
    run_tf_sample "ros2 run tf2_ros tf2_echo base_link laser_frame" ros2 run tf2_ros tf2_echo base_link laser_frame
    run_tf_sample "ros2 run tf2_ros tf2_echo odom base_link" ros2 run tf2_ros tf2_echo odom base_link
    run_tf_sample "ros2 run tf2_ros tf2_echo map odom" ros2 run tf2_ros tf2_echo map odom

    echo
    log_warn "测试 A: 让机器人原地逆时针转 90°，观察 odom yaw 和 scan/map 对齐情况。"
    read -r -p "完成测试 A 后按 Enter，继续测试 B..." _

    run_tf_sample "测试 A 后复查: tf2_echo odom base_link" ros2 run tf2_ros tf2_echo odom base_link
    run_tf_sample "测试 A 后复查: tf2_echo map odom" ros2 run tf2_ros tf2_echo map odom

    echo
    log_warn "测试 B: 让机器人直行 1m，观察 odom 连续性和 scan 是否持续贴着 map。"
    read -r -p "完成测试 B 后按 Enter，输出结束前复查..." _

    run_sample "测试 B 后复查: ros2 topic hz /odom" ros2 topic hz /odom
    run_sample "测试 B 后复查: ros2 topic hz /scan" ros2 topic hz /scan
    run_tf_sample "测试 B 后复查: tf2_echo odom base_link" ros2 run tf2_ros tf2_echo odom base_link
    run_tf_sample "测试 B 后复查: tf2_echo map odom" ros2 run tf2_ros tf2_echo map odom

    echo
    log_success "定位链路检查完成"
    print_notes
    cat <<'EOF'

建议复测命令:
  1. 点云方向不对时，先试:
     ./robot.sh navigation --localization-only --lidar-reversion --lidar-inverted
     ./robot.sh navigation --localization-only --lidar-reversion
     ./robot.sh navigation --localization-only --lidar-inverted
  2. 只有在驱动方向正常后，再试:
     ./robot.sh navigation --localization-only --lidar-yaw-deg 180
  3. 运动中 scan 不贴 map 时，再试:
     ./robot.sh navigation --localization-only --no-base-status-yaw
     ./robot.sh navigation --localization-only --base-odom-source wheel_cps
EOF
}

main "$@"
