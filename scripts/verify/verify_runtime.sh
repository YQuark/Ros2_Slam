#!/usr/bin/env bash
set -euo pipefail

required_topics=(
    /chassis/host_motion_command
    /chassis/link_state
    /chassis/host_control_state
    /chassis/firmware_control_state
    /chassis/firmware_info
    /wheel/observation
    /imu/observation
    /motion/supervision_state
    /platform/compatibility_state
    /wheel/odom
    /imu/data
    /odom
    /diagnostics
)

runtime_topics="$(ros2 topic list --no-daemon)"
for topic in "${required_topics[@]}"; do
    if ! grep -Fxq "$topic" <<<"$runtime_topics"; then
        echo "missing runtime topic: $topic" >&2
        exit 1
    fi
done

command_publishers="$(ros2 topic info /chassis/host_motion_command --verbose | awk '/Publisher count:/ {print $3}')"
if [ "$command_publishers" != "1" ]; then
    echo "chassis/host_motion_command must have exactly one publisher, got ${command_publishers:-unknown}" >&2
    exit 1
fi

for owned_topic in /wheel/odom /imu/data /odom /motion/supervision_state; do
    publisher_count="$(ros2 topic info "$owned_topic" --verbose | awk '/Publisher count:/ {print $3}')"
    if [ "$publisher_count" != "1" ]; then
        echo "$owned_topic must have exactly one publisher, got ${publisher_count:-unknown}" >&2
        exit 1
    fi
done

tf_output="$(timeout 5s ros2 run tf2_ros tf2_echo odom base_link 2>&1 || true)"
if ! grep -Fq "At time" <<<"$tf_output"; then
    echo "missing transform: odom -> base_link" >&2
    echo "$tf_output" >&2
    exit 1
fi
echo "Runtime contract OK"
