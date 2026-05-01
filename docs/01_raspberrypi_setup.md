# Raspberry Pi 环境

目标环境：

```text
Board: Raspberry Pi 4B
OS: Ubuntu 22.04 64-bit
ROS2: Humble
Workspace: /home/robot/ros2_ws
```

当前机器还没有 ROS2 环境时，先完成系统安装，再执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/robot/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

本仓库提供环境检查脚本：

```bash
./scripts/setup_ros_env.sh
./launch_scripts/robot.sh check
```

如果 `/opt/ros/humble/setup.bash` 不存在，脚本会直接报错，不会伪装为已安装 ROS2。
