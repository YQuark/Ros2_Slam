#!/bin/bash
# setup_rpi_performance.sh — 树莓派 4B 性能优化脚本
# 用于配置 CPU governor、GPU 内存、禁用不必要的服务
set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { printf '%b\n' "${YELLOW}$*${NC}"; }
log_ok() { printf '%b\n' "${GREEN}$*${NC}"; }
log_err() { printf '%b\n' "${RED}$*${NC}" >&2; }

if [ "$(uname -m)" != "aarch64" ]; then
    log_err "此脚本仅适用于 ARM64 平台（树莓派）"
    exit 1
fi

if [ "$(id -u)" -ne 0 ]; then
    log_err "请以 root 权限运行: sudo $0"
    exit 1
fi

log_info "=== 树莓派 4B 性能优化 ==="
echo ""

# 1. CPU governor → performance
log_info "1. 设置 CPU governor 为 performance"
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    if [ -f "$cpu" ]; then
        echo performance > "$cpu"
    fi
done
CURRENT=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
log_ok "  CPU governor: $CURRENT"
echo ""

# 2. GPU 内存 (减少到 128MB，为 ROS2 留更多 RAM)
log_info "2. GPU 内存配置"
BOOT_CONFIG="/boot/firmware/config.txt"
if [ ! -f "$BOOT_CONFIG" ]; then
    BOOT_CONFIG="/boot/config.txt"
fi
if [ -f "$BOOT_CONFIG" ]; then
    if grep -q "^gpu_mem=" "$BOOT_CONFIG"; then
        CURRENT_GPU=$(grep "^gpu_mem=" "$BOOT_CONFIG" | cut -d= -f2)
        log_info "  当前 GPU 内存: ${CURRENT_GPU}MB"
    else
        log_info "  未找到 gpu_mem 配置 (使用默认值)"
    fi
    log_info "  建议在 ${BOOT_CONFIG} 中添加: gpu_mem=128"
else
    log_err "  未找到 boot config 文件"
fi
echo ""

# 3. 禁用不必要的服务
log_info "3. 检查可禁用的服务"
DISABLE_SERVICES=(
    "bluetooth.service"
    "hciuart.service"
    "triggerhappy.service"
    "avahi-daemon.service"
)
for svc in "${DISABLE_SERVICES[@]}"; do
    if systemctl is-enabled "$svc" 2>/dev/null | grep -q "enabled"; then
        log_info "  禁用 $svc ..."
        systemctl disable "$svc" 2>/dev/null || true
        systemctl stop "$svc" 2>/dev/null || true
        log_ok "  已禁用 $svc"
    fi
done
echo ""

# 4. 温度检查
log_info "4. 当前温度"
if command -v vcgencmd >/dev/null 2>&1; then
    TEMP=$(vcgencmd measure_temp 2>/dev/null || echo "unknown")
    log_ok "  $TEMP"
    THROTTLED=$(vcgencmd get_throttled 2>/dev/null || echo "unknown")
    log_ok "  节流状态: $THROTTLED"
else
    log_info "  vcgencmd 不可用"
fi
echo ""

log_info "=== 优化完成 ==="
log_info "建议重启使 GPU 内存配置生效"
