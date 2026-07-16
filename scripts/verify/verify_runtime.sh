#!/usr/bin/env bash
set -euo pipefail

required_topics=(/chassis/status /chassis/command /chassis/state /chassis/firmware_info /diagnostics /odom /wheel/odom)

for topic in "${required_topics[@]}"; do
    if ! ros2 topic list --no-daemon | grep -Fxq "$topic"; then
        echo "missing runtime topic: $topic" >&2
        exit 1
    fi
done

command_publishers="$(ros2 topic info /chassis/command --verbose | awk '/Publisher count:/ {print $3}')"
if [ "$command_publishers" != "1" ]; then
    echo "chassis/command must have exactly one publisher, got ${command_publishers:-unknown}" >&2
    exit 1
fi

timeout 5s ros2 run tf2_ros tf2_echo odom base_link >/dev/null
echo "Runtime contract OK"
