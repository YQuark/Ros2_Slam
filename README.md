# ROS2 机器人上位机工程

本仓库负责把传感器接入、SLAM 建图、Nav2 导航、STM32 底盘桥接和可视化调试收敛成一套可维护、可复用、可扩展的 ROS2 工程体系。

## 能力矩阵

| 能力 | 状态 | 说明 |
| --- | --- | --- |
| 雷达建图 | 已支持 | `YDLIDAR X2 -> /scan -> slam_toolbox` |
| 摄像头建图 | 已支持 | `Astra Pro -> depthimage_to_laserscan -> slam_toolbox` |
| 真实底盘导航 | 已支持 | `Nav2 -> /cmd_vel -> stm32_robot_bridge -> STM32` |
| 虚拟底盘联调 | 已支持 | `base_mode:=fake` |
| 串口自动识别 | 已支持 | 自动区分底盘串口与雷达串口 |

## 统一入口

- 运维入口：`/home/robot/ros2_ws/launch_scripts/robot.sh`
- 技术总入口：`/home/robot/ros2_ws/src/robot_bringup/launch/system.launch.py`

## 快速开始

```bash
cd /home/robot/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source /home/robot/ros2_ws/install/setup.bash
```

```bash
cd /home/robot/ros2_ws/launch_scripts
./robot.sh mapping lidar --real-base
./robot.sh save-map my_map
./robot.sh navigation /home/robot/ros2_maps/my_map.yaml --real-base
```

## 目录结构

```text
ros2_ws
├── docs/                  # 中文文档体系
├── launch_scripts/        # 统一运维入口与诊断脚本
├── src/
│   ├── robot_bringup      # launch / config / rviz 总编排
│   ├── stm32_robot_bridge # 底盘桥接
│   ├── ydlidar_ros2_driver
│   ├── ros2_astra_camera
│   └── YDLidar-SDK
├── README.md
└── SYSTEM_OVERVIEW.md
```

## 文档导航

- [文档总览](./docs/README.md)
- [快速开始](./docs/01-快速开始.md)
- [系统架构](./docs/02-系统架构.md)
- [硬件接线与设备识别](./docs/03-硬件接线与设备识别.md)
- [建图指南](./docs/04-建图指南.md)
- [导航指南](./docs/05-导航指南.md)
- [底盘与串口桥接](./docs/06-底盘与串口桥接.md)
- [运维与排障](./docs/07-运维与排障.md)
- [开发与二次复用](./docs/08-开发与二次复用.md)
- [迁移与兼容说明](./docs/09-迁移与兼容说明.md)

## 设计原则

- 技术编排只保留一个核心入口：`src/robot_bringup/launch/system.launch.py`
- 运维层只保留一个推荐 CLI：`launch_scripts/robot.sh`
- 历史 `start_*.sh` 继续可用，但仅作为兼容包装
- 文档统一使用中文，并按用户路径与开发路径拆分
