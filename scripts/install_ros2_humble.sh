#!/bin/bash

set -euo pipefail

if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

if ! command -v apt >/dev/null 2>&1; then
    echo "仅支持 Ubuntu/Debian apt 环境。" >&2
    exit 1
fi

. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "jammy" ]; then
    echo "当前系统不是 Ubuntu 22.04 jammy: ${PRETTY_NAME:-unknown}" >&2
    exit 1
fi

ARCH="$(dpkg --print-architecture)"
if [ "$ARCH" != "arm64" ] && [ "$ARCH" != "amd64" ]; then
    echo "ROS2 Humble deb 目标平台应为 arm64 或 amd64，当前为: $ARCH" >&2
    exit 1
fi

export DEBIAN_FRONTEND=noninteractive
export LANG=en_US.UTF-8

echo "==> 安装基础工具和 locale"
$SUDO apt update
$SUDO apt install -y locales software-properties-common curl ca-certificates gnupg lsb-release
$SUDO locale-gen en_US en_US.UTF-8
$SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "==> 启用 Ubuntu universe 仓库"
$SUDO add-apt-repository -y universe

echo "==> 安装 ROS apt source 配置包"
ROS_APT_SOURCE_VERSION="$(
    curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F "tag_name" \
        | awk -F'"' '{print $4}'
)"
if [ -z "$ROS_APT_SOURCE_VERSION" ]; then
    echo "无法获取 ros-apt-source 最新版本。" >&2
    exit 1
fi

ROS_APT_SOURCE_DEB="/tmp/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
curl -L -o "$ROS_APT_SOURCE_DEB" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
$SUDO dpkg -i "$ROS_APT_SOURCE_DEB"

echo "==> 更新 apt 并升级 systemd/udev 相关基础包"
$SUDO apt update
$SUDO apt upgrade -y

echo "==> 安装 ROS2 Humble 与开发工具"
$SUDO apt install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete \
    build-essential \
    cmake \
    git \
    pkg-config \
    swig \
    libudev-dev \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-xacro

echo "==> 初始化 rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    $SUDO rosdep init
fi
rosdep update

echo "==> 配置当前用户 bash 环境"
BASHRC="${HOME}/.bashrc"
if ! grep -Fq "source /opt/ros/humble/setup.bash" "$BASHRC"; then
    echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
fi
if ! grep -Fq "source /home/robot/ros2_ws/install/setup.bash" "$BASHRC"; then
    cat >> "$BASHRC" <<'EOF'
if [ -f /home/robot/ros2_ws/install/setup.bash ]; then
    source /home/robot/ros2_ws/install/setup.bash
fi
EOF
fi

echo "==> 验证 ROS2"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
ros2 --version
colcon version-check || true

echo "ROS2 Humble 安装完成。下一步:"
echo "  cd /home/robot/ros2_ws"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo "  colcon build --symlink-install"
