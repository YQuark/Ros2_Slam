#!/usr/bin/env bash
set -euo pipefail

required_topics=(/chassis/status /chassis/command /odom /wheel/odom)

for topic in "${required_topics[@]}"; do
    if ! ros2 topic list --no-daemon | grep -Fxq "$topic"; then
        echo "missing runtime topic: $topic" >&2
        exit 1
    fi
done

timeout 5s ros2 run tf2_ros tf2_echo odom base_link >/dev/null
echo "Runtime contract OK"
