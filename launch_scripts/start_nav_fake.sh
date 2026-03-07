#!/bin/bash
# ============================================
# 虚拟底盘导航启动脚本
# 功能：雷达 + 虚拟底盘 + 定位 + Nav2 + RViz
# ============================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

HAS_FAKE=false
for arg in "$@"; do
    if [ "$arg" = "--fake-base" ]; then
        HAS_FAKE=true
        break
    fi
done

if [ "$HAS_FAKE" = true ]; then
    exec "$SCRIPT_DIR/start_navigation.sh" "$@"
else
    exec "$SCRIPT_DIR/start_navigation.sh" "$@" --fake-base
fi
