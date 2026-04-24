#!/bin/bash

set -u

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/common.sh"

TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-3}"

print_topic_status() {
    local topic_name="$1"
    local label="$2"

    if ros2 topic list --no-daemon 2>/dev/null | grep -Fxq "$topic_name"; then
        log_success "✓ ${label}: ${topic_name}"
    else
        log_warn "⚠ ${label}: 缺少 ${topic_name}"
    fi
}

print_lifecycle_status() {
    local node_name="$1"
    local label="$2"
    local state=""
    local rc=0

    if ! ros2_node_exists "$node_name"; then
        log_error "✗ ${label}: 未检测到节点 ${node_name}"
        return 1
    fi

    state="$(ros2_lifecycle_get_state "$node_name" 3)"
    rc=$?
    if [ "$rc" -eq 124 ]; then
        log_warn "⚠ ${label}: ${node_name} 生命周期查询超时，节点可能卡住或未响应"
        return 1
    fi
    if [ -z "$state" ]; then
        log_warn "⚠ ${label}: ${node_name} 生命周期状态未知"
        return 1
    fi
    if printf '%s\n' "$state" | grep -qi "active"; then
        log_success "✓ ${label}: ${node_name} active"
        return 0
    fi

    log_warn "⚠ ${label}: ${node_name} 当前状态: ${state:-unknown}"
    return 1
}

wait_for_topic_once() {
    local topic_name="$1"
    local label="$2"

    case "$(ros2_wait_for_topic_message "$topic_name" "$TOPIC_WAIT_SEC"; printf '%s' "$?")" in
        0)
            log_success "✓ ${label}: ${topic_name} 在 ${TOPIC_WAIT_SEC}s 内收到消息"
            return 0
            ;;
        2)
            log_warn "⚠ ${label}: 系统缺少 timeout，跳过消息等待"
            return 0
            ;;
        *)
            log_warn "⚠ ${label}: ${topic_name} 在 ${TOPIC_WAIT_SEC}s 内没有消息"
            return 1
            ;;
    esac
}

print_action_status() {
    local action_name="$1"
    local label="$2"

    if ros2 action list --no-daemon 2>/dev/null | grep -Fxq "$action_name"; then
        log_success "✓ ${label}: ${action_name}"
    else
        log_warn "⚠ ${label}: 缺少 ${action_name}"
    fi
}

ensure_workspace_built
setup_ros_env

print_header "导航链路检查"

log_info "1. 生命周期节点"
print_lifecycle_status "/map_server" "地图服务"
print_lifecycle_status "/amcl" "AMCL"
print_lifecycle_status "/planner_server" "全局规划器"
print_lifecycle_status "/controller_server" "局部控制器"
print_lifecycle_status "/bt_navigator" "行为树导航器"
echo

log_info "2. 动作与话题入口"
print_action_status "/navigate_to_pose" "导航动作"
print_action_status "/follow_path" "路径跟踪动作"
print_action_status "/compute_path_to_pose" "路径规划动作"
print_topic_status "/goal_pose" "RViz 目标输入"
print_topic_status "/initialpose" "RViz 初始位姿输入"
echo

log_info "3. 核心运行话题"
print_topic_status "/map" "静态地图"
print_topic_status "/scan" "激光数据"
print_topic_status "/odom" "里程计"
print_topic_status "/amcl_pose" "定位输出"
print_topic_status "/plan" "全局路径"
print_topic_status "/local_plan" "局部路径"
print_topic_status "/cmd_vel" "底盘速度命令"
echo

log_info "4. 实时消息检查"
wait_for_topic_once "/amcl_pose" "定位链"
wait_for_topic_once "/odom" "底盘里程计"
wait_for_topic_once "/scan" "雷达数据"
echo

log_info "5. 排障结论"
if ! ros2_lifecycle_is_active "/bt_navigator"; then
    log_error "✗ bt_navigator 未 active。先查 Nav2 生命周期是否完整启动。"
elif ! ros2 action list --no-daemon 2>/dev/null | grep -Fxq "/navigate_to_pose"; then
    log_error "✗ /navigate_to_pose action 不存在。Nav2 目标入口未就绪。"
elif ! ros2 topic list --no-daemon 2>/dev/null | grep -Fxq "/goal_pose"; then
    log_error "✗ /goal_pose 话题不存在。RViz 2D Goal Pose 不会进入导航链。"
else
    log_success "✓ 导航入口基本存在。"
    log_info "如果你刚点过 2D Goal Pose 但仍看不到路径，请按下面顺序看："
    echo "  1. ros2 topic echo /goal_pose --once"
    echo "  2. ros2 topic echo /plan --once"
    echo "  3. ros2 topic echo /local_plan --once"
    echo "  4. ros2 topic hz /cmd_vel"
fi
echo

log_info "6. 典型分流"
echo "  • /goal_pose 没消息: 先查 RViz 2D Goal Pose 是否真的发出目标"
echo "  • /goal_pose 有消息但 /plan 没有: 查 bt_navigator / planner_server / 全局 costmap"
echo "  • /plan 有消息但 /local_plan 没有: 查 controller_server / 局部 costmap"
echo "  • /cmd_vel 有消息但车不动: 查 stm32_bridge / 串口 / 底盘上电 / /odom"
