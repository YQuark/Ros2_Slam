# 启动与运维入口

`launch_scripts/` 现在分为两层：

- 统一入口：`robot.sh`
- 兼容入口：历史 `start_*.sh` 脚本，内部已收编为 `robot.sh` 的包装层

## 推荐入口

```bash
cd /home/robot/ros2_ws/launch_scripts
./robot.sh --help
```

常用命令：

```bash
./robot.sh mapping camera
./robot.sh mapping lidar --real-base
./robot.sh mapping lidar --real-base --ekf-base
./robot.sh mapping lidar precision --real-base --ekf-base
./robot.sh navigation /home/robot/ros2_maps/my_map.yaml --real-base
./robot.sh navigation /home/robot/ros2_maps/my_map.yaml --real-base --ekf-base
./robot.sh navigation /home/robot/ros2_maps/my_map.yaml --fake-base
./robot.sh sensor lidar
./robot.sh sensor camera
./robot.sh base
./robot.sh full
./robot.sh check lidar
./robot.sh check mapping
./robot.sh save-map my_map
./robot.sh doctor
./robot.sh teleop
./robot.sh stop
```

## 目录分层

- `robot.sh`
  统一 CLI，总控建图、导航、传感器、底盘和健康检查入口。
- `lib/common.sh`
  启动脚本共享函数，包括 ROS 环境加载、日志输出和 YAML/串口辅助逻辑。
- `start_*.sh`
  历史兼容包装，内部直接转发到 `robot.sh`。
- `check_*.sh`、`detect_*.sh`
  诊断与设备探测工具，供 `robot.sh` 和运维排障复用。

## 当前推荐组合

- 真实底盘雷达建图默认入口：`./robot.sh mapping lidar --real-base`
- 需要底盘 EKF 融合时：`./robot.sh mapping lidar --real-base --ekf-base`
- 精细雷达建图档：`./robot.sh mapping lidar precision --real-base --ekf-base`

补充说明：

- `--ekf-base` 会启用 `/odom + /imu/data -> /odometry/filtered` 的底盘融合链路。
- `precision` 档位当前定义为在 `quality` 行为基础上保留更细的 `0.03m` 栅格分辨率，不再单独收紧一组激进门限。
- 底盘口自动探测会优先对候选 CP210x 串口发送 `GET_STATUS` 进行活体探测，而不是只按 `/dev` 名称猜测。

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

这些脚本仍可独立使用，同时也可以通过 `robot.sh` 间接调用。
