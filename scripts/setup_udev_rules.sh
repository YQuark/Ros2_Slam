#!/bin/bash

set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-yquark-slam-robot.rules"

if [ "$(id -u)" -ne 0 ]; then
    echo "请使用 sudo 运行: sudo $0" >&2
    exit 1
fi

cat > "$RULE_FILE" <<'EOF'
# YQuark ROS2 SLAM robot device aliases.
# 根据现场 lsusb/udevadm 信息补充 idVendor/idProduct/serial，避免误匹配。
KERNEL=="ttyUSB*", ATTRS{idVendor}=="待实测", ATTRS{idProduct}=="待实测", SYMLINK+="ydlidar", MODE="0666", GROUP="dialout"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="待实测", ATTRS{idProduct}=="待实测", SYMLINK+="stm32_chassis", MODE="0666", GROUP="dialout"
EOF

udevadm control --reload-rules
udevadm trigger

echo "已写入: $RULE_FILE"
echo "请重新插拔 YDLIDAR X2 和 STM32 USB-UART 后检查 /dev/ydlidar 与 /dev/stm32_chassis。"
