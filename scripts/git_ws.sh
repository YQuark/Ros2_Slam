#!/bin/bash

set -euo pipefail

ROS_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec git --git-dir="${ROS_WS}/.git-real" --work-tree="${ROS_WS}" "$@"
