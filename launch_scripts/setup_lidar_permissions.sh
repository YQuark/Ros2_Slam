#!/bin/bash

set -euo pipefail

USER_NAME="${SUDO_USER:-${USER:-robot}}"
RULE_FILE="/etc/udev/rules.d/99-ydlidar.rules"

if [ "$(id -u)" -ne 0 ]; then
    echo "请用 sudo 运行:"
    echo "  sudo $0"
    exit 1
fi

usermod -aG dialout "$USER_NAME"

cat > "$RULE_FILE" <<'EOF'
# YDLIDAR X2 commonly uses a CP210x USB-to-UART bridge.
# Keep a stable /dev/ydlidar name and allow dialout users to access it.
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ydlidar", GROUP="dialout", MODE="0660"
EOF

udevadm control --reload-rules
udevadm trigger

if [ -e /dev/ttyUSB0 ]; then
    chgrp dialout /dev/ttyUSB0 || true
    chmod 660 /dev/ttyUSB0 || true
fi

echo "雷达串口权限已配置。"
echo "用户 ${USER_NAME} 已加入 dialout。请重新登录或重启后再运行雷达驱动。"
