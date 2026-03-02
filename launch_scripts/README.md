# 机器人启动脚本说明

## 目录

`/home/robot/ros2_ws/launch_scripts`

- `start_mapping.sh`：建图模式（可选 `camera` 或 `lidar`）
- `start_navigation.sh`：导航模式（需要已有地图）
- `start_full_system.sh`：完整系统（雷达 + Astra + 底盘 + SLAM）
- `start_lidar.sh`：仅雷达与可视化
- `start_camera.sh`：仅 Astra Pro 摄像头
- `start_controller.sh`：仅底盘串口桥接
- `save_map.sh`：保存 SLAM 地图
- `stop_all.sh`：停止 ROS2 相关进程
- `check_system.sh`：基础环境检查
- `test_camera.sh`：Astra 驱动自检

## 推荐用法

1. 摄像头建图
```bash
cd /home/robot/ros2_ws/launch_scripts
./start_mapping.sh camera
```

2. 雷达建图
```bash
./start_mapping.sh lidar
```

3. 保存地图
```bash
./save_map.sh my_map
```

4. 导航
```bash
./start_navigation.sh /home/robot/ros2_maps/my_map.yaml
```

## 注意

- 不要同时启动多个雷达 launch，避免串口抢占导致 `Check Sum` 错误。
- 没有底盘里程计时，建图请使用 `use_base:=false`（已是默认）。
- 如果要接底盘，请确认端口 `base_port:=/dev/ttyUSB1` 正确。
