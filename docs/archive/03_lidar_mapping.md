# YDLIDAR X2 与建图入口

雷达单独启动：

```bash
./launch_scripts/robot.sh sensor lidar
```

覆盖串口：

```bash
./launch_scripts/robot.sh sensor lidar --lidar-port /dev/ydlidar
```

雷达检查：

```bash
./launch_scripts/robot.sh check lidar
./launch_scripts/check_lidar.sh --lidar-port /dev/ydlidar
```

无头建图入口：

```bash
./launch_scripts/robot.sh mapping lidar --manual --real-base --no-rviz
```

雷达方向必须现场确认：

- 机器人正前方障碍物在 RViz 中位于 `base_link` 正前方
- 原地旋转时点云和地图方向一致
- 建图正确但导航反向时，优先查 TF yaw、odom yaw 和底盘方向符号
