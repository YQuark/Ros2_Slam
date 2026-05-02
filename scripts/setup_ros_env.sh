#!/bin/bash

set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
ROS_WS="${ROS_WS:-/home/robot/ros2_ws}"

if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS2 环境不存在: $ROS_SETUP" >&2
    echo "请先在 Ubuntu 22.04 arm64 上安装 ROS2 Humble，再 source 本脚本。" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"

if [ -f "${ROS_WS}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${ROS_WS}/install/setup.bash"
fi

echo "ROS_DISTRO=${ROS_DISTRO}"
echo "ROS_WS=${ROS_WS}"
