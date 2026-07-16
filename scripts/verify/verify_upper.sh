#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/humble/setup.bash
set -u

colcon build --base-paths src --symlink-install --packages-skip ydlidar_ros2_driver
set +u
source install/setup.bash
set -u
colcon test --base-paths src --packages-skip ydlidar_ros2_driver --return-code-on-test-failure
colcon test-result --verbose
python3 -m pytest tests/ src/ -q
python3 scripts/verify/check_repo_boundaries.py
