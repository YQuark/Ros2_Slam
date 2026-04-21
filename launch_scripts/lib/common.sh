#!/bin/bash

ROS_WS="${ROS_WS:-/home/robot/ros2_ws}"
SCRIPT_DIR="${SCRIPT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    local title="$1"
    printf '%b\n' "${BLUE}========================================${NC}"
    printf '%b\n' "${BLUE}    ${title}${NC}"
    printf '%b\n' "${BLUE}========================================${NC}"
    printf '\n'
}

log_info() {
    printf '%b\n' "${YELLOW}$*${NC}"
}

log_success() {
    printf '%b\n' "${GREEN}$*${NC}"
}

log_warn() {
    printf '%b\n' "${YELLOW}$*${NC}"
}

log_error() {
    printf '%b\n' "${RED}$*${NC}" >&2
}

setup_ros_env() {
    local restore_nounset=0

    case $- in
        *u*)
            restore_nounset=1
            set +u
            ;;
    esac

    # shellcheck disable=SC1091
    source /opt/ros/foxy/setup.bash
    if [ -f "${ROS_WS}/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "${ROS_WS}/install/setup.bash"
    fi

    if [ "$restore_nounset" -eq 1 ]; then
        set -u
    fi
}

ensure_workspace_built() {
    if [ ! -f "${ROS_WS}/install/setup.bash" ]; then
        log_error "✗ 工作空间未构建: ${ROS_WS}/install/setup.bash"
        log_warn "请先执行: cd ${ROS_WS} && colcon build --symlink-install"
        exit 1
    fi
}

resolve_port() {
    local candidate="$1"
    readlink -f "$candidate" 2>/dev/null || printf '%s\n' "$candidate"
}

get_yaml_key_value() {
    local yaml_file="$1"
    local key="$2"
    awk -v key="${key}:" '$1 == key { print $2; exit }' "$yaml_file"
}

render_yaml_key_value() {
    local src="$1"
    local key="$2"
    local value="$3"
    local dst="$4"

    sed -E "s|^([[:space:]]*${key}:).*|\\1 ${value}|" "$src" > "$dst"
}

ros2_node_exists() {
    local node_name="$1"
    ros2 node list 2>/dev/null | grep -Fxq "$node_name"
}

ros2_lifecycle_is_active() {
    local node_name="$1"
    ros2 lifecycle get "$node_name" 2>/dev/null | grep -qi "active"
}

ros2_wait_for_topic_message() {
    local topic_name="$1"
    local timeout_sec="${2:-5}"

    if ! command -v timeout >/dev/null 2>&1; then
        return 2
    fi

    timeout "${timeout_sec}s" ros2 topic echo --once "$topic_name" >/dev/null 2>&1
}
