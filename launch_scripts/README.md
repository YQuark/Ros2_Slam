# 启动与运维入口

`launch_scripts/` 现在只保留一套正式入口：

- 正式入口：`robot.sh`
- 兼容入口：历史 `start_*.sh`，仅做转发

## 推荐入口

```bash
cd /home/robot/ros2_ws/launch_scripts
./robot.sh --help
```

## 当前推荐流程

主业务流程：

```bash
./robot.sh mapping lidar --real-base --ekf-base
./robot.sh save-map my_map
./robot.sh navigation --real-base --ekf-base
```

运行 `navigation` 后：

1. 先在 RViz 中执行 `2D Pose Estimate`
2. 确认激光与地图墙体基本重合
3. 再用 `2D Goal Pose` 发目标

两阶段导航只在定位未就绪或异常时使用：

```bash
./robot.sh navigation --real-base --ekf-base --localization-only
./robot.sh navigation --nav2-only
```

## 常用命令

```bash
./robot.sh mapping camera
./robot.sh mapping lidar --real-base
./robot.sh mapping lidar precision --real-base --ekf-base
./robot.sh save-map my_map
./robot.sh navigation --real-base --ekf-base
./robot.sh navigation /home/robot/ros2_maps/my_map.yaml --fake-base
./robot.sh sensor lidar
./robot.sh sensor camera
./robot.sh base
./robot.sh full
./robot.sh check lidar
./robot.sh check mapping
./robot.sh check navigation
./robot.sh doctor
./robot.sh teleop
./robot.sh stop
```

## 入口约定

- `robot.sh`
  统一 CLI，总控建图、导航、传感器、底盘和健康检查。
- `lib/common.sh`
  启动脚本共享函数。
- `start_*.sh`
  兼容壳，内部直接转发到 `robot.sh`。
- `check_*.sh`、`detect_*.sh`
  诊断与设备探测工具。

## 兼容脚本状态

- `start_mapping.sh` -> `robot.sh mapping`
- `start_navigation.sh` -> `robot.sh navigation`
- `start_nav_fake.sh` -> `robot.sh navigation --fake-base`
- `start_lidar.sh` -> `robot.sh sensor lidar`
- `start_camera.sh` -> `robot.sh sensor camera`
- `start_controller.sh` -> `robot.sh base`
- `start_full_system.sh` -> `robot.sh full`

## 保留的运维工具

- `check_lidar_health.sh`
- `check_mapping_pipeline.sh`
- `check_system.sh`
- `save_map.sh`
- `keyboard_control.sh`
- `stop_all.sh`

这些脚本仍可独立使用，但正式用户流程仍以 `robot.sh` 为准。
