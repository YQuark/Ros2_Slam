#!/bin/bash
set -euo pipefail

source_ros_setup() {
    local restore_nounset=0

    case $- in
        *u*)
            restore_nounset=1
            set +u
            ;;
    esac

    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash

    if [ "$restore_nounset" -eq 1 ]; then
        set -u
    fi
}

source_ros_setup
export CMAKE_PREFIX_PATH="/opt/slamrobot/vendor/ydlidar-sdk:${CMAKE_PREFIX_PATH:-}"
rosdep install --from-paths src --ignore-src -r -y --skip-keys ydlidar_sdk
colcon build --base-paths src --symlink-install
colcon test --base-paths src --packages-skip ydlidar_ros2_driver --return-code-on-test-failure
colcon test-result --verbose
