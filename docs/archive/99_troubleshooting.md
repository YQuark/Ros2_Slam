# 排障顺序

雷达异常：

1. `/dev/ydlidar` 是否存在
2. `./launch_scripts/robot.sh check lidar` 是否通过
3. `/scan` 是否有数据和稳定频率
4. `base_link -> laser_frame` TF 是否存在
5. 再考虑调整 `inverted`、`reversion` 或 `lidar_tf_yaw`

环境异常：

1. `/opt/ros/humble/setup.bash` 是否存在
2. `colcon` 是否已安装
3. `rosdep install --from-paths src --ignore-src -r -y` 是否完成
4. `colcon build --symlink-install` 是否通过

当前没有 ROS2 环境时，不判断 `/scan`、Nav2 或 slam_toolbox 行为。
