# 启动与运维入口

`launch_scripts/` 在 v0.4.x 仅保留兼容入口：

- 正式入口：`bin/robot`
- 兼容入口：历史 `start_*.sh`，仅做转发

## 推荐入口

```bash
cd /home/robot/ros2_ws
./bin/robot --help
```

## 雷达 RViz 建图

统一入口只使用 `bin/robot`。树莓派现场先用虚拟底盘 TF 跑通雷达、SLAM Toolbox 和 RViz：

```bash
cd /home/robot/ros2_ws
./bin/robot mapping lidar --manual --fake-base
```

需要真实底盘里程计时：

```bash
./bin/robot mapping lidar --manual --real-base --base-port auto
```

如需手动指定雷达串口，可追加 `--lidar-port /dev/ttyUSB0`。

RViz 配置使用 `src/robot_bringup/rviz/lidar_mapping.rviz`，固定坐标系为 `map`，显示 `/map`、`/map_points`、`/scan` 和 TF。

## 当前推荐流程

主业务流程：

```bash
./bin/robot mapping lidar --real-base --ekf-base
./bin/robot save-map my_map
./bin/robot navigation --real-base --ekf-base
```

运行 `navigation` 后：

1. 先在 RViz 中执行 `2D Pose Estimate`
2. 确认激光与地图墙体基本重合
3. 再用 `2D Goal Pose` 发目标

两阶段导航只在定位未就绪或异常时使用：

```bash
./bin/robot navigation --real-base --ekf-base --localization-only
./bin/robot navigation --nav2-only
```

## 常用命令

```bash
./bin/robot mapping camera
./bin/robot mapping lidar --real-base
./bin/robot mapping lidar precision --real-base --ekf-base
./bin/robot save-map my_map
./bin/robot navigation --real-base --ekf-base
./bin/robot navigation /home/robot/ros2_maps/my_map.yaml --fake-base
./bin/robot sensor lidar
./bin/robot sensor camera
./bin/robot base
./bin/robot full
./bin/robot check lidar
./bin/robot check mapping
./bin/robot check navigation
./bin/robot doctor
./bin/robot teleop
./bin/robot stop
```

## 入口约定

- `bin/robot`
  统一 CLI，总控建图、导航、传感器、底盘和健康检查。
- `lib/common.sh`
  启动脚本共享函数。
- `start_*.sh`
  兼容壳，内部直接转发到 `bin/robot`。
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

这些脚本仍可独立使用，但正式用户流程仍以 `bin/robot` 为准。
