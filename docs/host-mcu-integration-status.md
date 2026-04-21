# 上位机与下位机关联状态总览

日期：2026-04-12

本文档是当前 ROS2 上位机与 STM32/ESP01S 下位机联调状态的稳定入口。旧的中文专题文档在当前 SMB/终端环境中存在文件名编码显示异常，因此后续排查优先从本文档进入，再回溯到 `docs/requirements/`、`docs/plans/` 和 `outputs/runtime/vibe-sessions/`。

## 结论摘要

当前已静态修正的系统级问题集中在六条链路：

- 串口资源：固定默认底盘 `/dev/ttyUSB1`、雷达 `/dev/ttyUSB0`，并让底盘自动探测排除雷达串口，避免 YDLIDAR 与 STM32 同抢一个口。
- 速度尺度：上位机 `max_linear/max_angular` 已按下位机 `MAX_CPS`、轮半径、编码器计数和轮距计算，保持 Q15 命令与反馈尺度一致。
- 航向来源：上位机默认使用 STM32 `yaw_est` 作为 `/odom` 航向来源，不再长期依赖桥接层自行积分角速度。
- 里程计安全：STM32 状态过期时，上位机 `/odom` 不再回退到 `/cmd_vel` 命令速度，而是保持位姿、速度置零并提高协方差。
- IMU 加速度：下位机在静止校准时加入体坐标重力方向检查；上位机按有效、原始可观测、无效三种语义发布不同协方差。
- 激光手性：`/scan` 相对 `base_link` 的左右镜像修复收口到雷达参数 `inverted: true`，并允许在 `robot.sh` 里做运行时覆盖。

所有运行结论仍需在远程机器人环境验证。当前本地目录为 SMB 挂载，只允许静态阅读和编辑。

## 关键文件关联

上位机 ROS2：

- `src/stm32_robot_bridge/stm32_robot_bridge/bridge_node.py`
- `src/stm32_robot_bridge/launch/stm32_bridge.launch.py`
- `src/robot_bringup/launch/base.launch.py`
- `src/robot_bringup/launch/system.launch.py`
- `src/robot_bringup/config/nav2_params_robot.yaml`
- `src/robot_bringup/config/ydlidar_X2_mapping.yaml`
- `src/ydlidar_ros2_driver/params/X2.yaml`
- `launch_scripts/robot.sh`
- `launch_scripts/detect_base_port.sh`

下位机 STM32/ESP01S：

- `/mnt/d/Document/Work/projects/Clion/Core/Inc/Drivers/robot_config.h`
- `/mnt/d/Document/Work/projects/Clion/Core/Src/Drivers/robot_control.c`
- `/mnt/d/Document/Work/projects/Clion/Core/Src/Drivers/mpu6050.c`
- `/mnt/d/Document/Work/projects/Clion/Core/Src/Drivers/link_proto.c`
- `/mnt/d/Document/Work/projects/Clion/ESP01S`

## 串口与启动契约

当前默认约定：

- STM32 底盘：`/dev/ttyUSB1`
- YDLIDAR X2：`/dev/ttyUSB0`
- `system.launch.py` 默认 `base_port:=/dev/ttyUSB1`
- `ydlidar_X2_mapping.yaml` 默认 `port: /dev/ttyUSB0`
- `bridge_node.py` 支持 `excluded_ports`，`system.launch.py` 会把雷达串口传给底盘桥接，避免自动探测误选雷达。

排查原则：

- 不再仅凭 CP2102 设备名判断底盘串口。
- 若必须自动探测，底盘探测必须排除已知雷达串口。
- 现场如改线，优先改 launch 参数或 YAML，不要让两个节点同时打开同一串口。

## 激光手性与外参契约

当前默认约定：

- `src/robot_bringup/config/ydlidar_X2_mapping.yaml` 使用 `inverted: true`
- `reversion` 保持 `false`
- `base_link -> laser_frame` 默认 `yaw=1.570796326795 rad`
- `./robot.sh mapping` 和 `./robot.sh navigation` 支持 `--lidar-reversion`、`--lidar-inverted`、`--lidar-yaw-rad/deg` 运行时覆盖

排查原则：

- 在 RViz 中 `Fixed Frame=base_link` 且只看 `LaserScan` 时，左右镜像优先判定为激光坐标链问题。
- `BASE_USE_STATUS_YAW`、`BASE_ODOM_FEEDBACK_SOURCE`、`BASE_ODOM_ANGULAR_SIGN` 属于 odom 链，不是修正 scan 左右镜像的首选入口。
- 如果历史地图是在错误 scan 手性下建立，地图应废弃并重建。

## 速度尺度契约

下位机物理尺度来源：

- `MAX_CPS = 13800`
- `DIFF_WHEEL_RADIUS_M = 0.0325`
- `ENCODER_COUNTS_PER_REV = 2340`
- `DIFF_TRACK_WIDTH_M = 0.125`

由此得到上位机协议尺度：

- `max_linear = 1.204277 m/s`
- `max_angular = 19.268435 rad/s`

该尺度只用于 Q15 协议编码/解码，不等于导航运行速度。Nav2 仍应使用独立安全限速，例如 `nav2_params_robot.yaml` 中的较低 `max_vel_x` 和 `max_vel_theta`。

相关依据：

- 上位机编码：`bridge_node.py` 中 `/cmd_vel -> Q15`
- 上位机解码：`bridge_node.py` 中 `v_est_q15/w_est_q15 -> /odom`
- 下位机解释：`robot_control.c` 中 Q15/归一化速度到轮速参考
- 下位机参数：`robot_config.h`

## 航向、IMU 与 EKF

当前航向链路：

- STM32 通过 IMU 和编码器状态导出 `yaw_est`
- 上位机 `use_status_yaw=true` 时使用 `yaw_est` 更新 `/odom` 航向
- 上位机发布 `/imu/data`
- `base_fusion_mode:=ekf` 时使用 `robot_localization` 输出 `/odometry/filtered`

加速度语义：

- `/imu/data.linear_acceleration` 是下位机映射后的体坐标原始加速度，不是去重力后的平移加速度。
- 下位机 `imu_accel_valid` 不能只看模长接近 `1g`，还必须满足当前安装方向下的体坐标重力方向约束。
- 上位机按 `valid_accel_covariance`、`raw_accel_covariance`、`invalid_accel_covariance` 区分可信等级。

机械安装前提：

- 底盘主板平行于地面。
- MPU6050 垂直插在主板上。
- 已有静态数据支持“车体竖直主分量来自当前映射后的体坐标 Z”这一方向检查，但最终仍需实机姿态旋转测试确认符号和轴向。

## 里程计安全策略

上位机 `/odom` 必须是反馈测量，不是命令回显。

当前策略：

- STM32 状态新鲜时，`/odom.twist` 使用 `feedback_vx/feedback_wz`。
- STM32 状态过期、串口异常或未收到状态时，`/odom.twist` 置零。
- 状态过期时保持最后位姿，不积分 `/cmd_vel`。
- 状态过期时提高 `/odom` 协方差，避免 EKF/SLAM 把失联期间的里程计当作可信运动。

该策略解决的问题：

- 防止串口异常时 `/odom` 继续按命令速度漂移。
- 防止下位机安全逻辑已经抑制运动时，上位机仍制造虚假位姿。
- 防止假位姿污染 `map -> odom`、EKF、SLAM 和导航闭环。

## 文档索引

需求冻结：

- `docs/requirements/2026-04-09-heading-localization-chassis-audit.md`
- `docs/requirements/2026-04-10-firmware-imu-mounting-and-accel-validity.md`
- `docs/requirements/2026-04-11-host-firmware-velocity-scale-contract.md`
- `docs/requirements/2026-04-11-manual-control-arbitration-and-yaw-hold.md`
- `docs/requirements/2026-04-12-esp01s-control-stutter.md`
- `docs/requirements/2026-04-12-manual-yaw-hold-isolation.md`
- `docs/requirements/2026-04-11-odom-stale-command-fallback.md`
- `docs/requirements/2026-04-11-full-host-mcu-code-audit.md`
- `docs/requirements/2026-04-11-full-host-mcu-code-reaudit.md`

执行计划：

- `docs/plans/2026-04-09-heading-localization-chassis-audit-execution-plan.md`
- `docs/plans/2026-04-10-firmware-imu-mounting-and-accel-validity-execution-plan.md`
- `docs/plans/2026-04-11-host-firmware-velocity-scale-contract-execution-plan.md`
- `docs/plans/2026-04-11-manual-control-arbitration-and-yaw-hold-execution-plan.md`
- `docs/plans/2026-04-12-esp01s-control-stutter-execution-plan.md`
- `docs/plans/2026-04-12-manual-yaw-hold-isolation-execution-plan.md`
- `docs/plans/2026-04-11-odom-stale-command-fallback-execution-plan.md`
- `docs/plans/2026-04-11-full-host-mcu-code-audit-execution-plan.md`
- `docs/plans/2026-04-11-full-host-mcu-code-reaudit-execution-plan.md`

运行回执：

- `outputs/runtime/vibe-sessions/2026-04-09-heading-audit`
- `outputs/runtime/vibe-sessions/2026-04-10-firmware-imu-mounting-audit`
- `outputs/runtime/vibe-sessions/2026-04-11-velocity-scale-contract`
- `outputs/runtime/vibe-sessions/2026-04-11-manual-control-regression`
- `outputs/runtime/vibe-sessions/2026-04-12-esp01s-control-stutter`
- `outputs/runtime/vibe-sessions/2026-04-12-manual-yaw-hold-isolation`
- `outputs/runtime/vibe-sessions/2026-04-11-odom-stale-command-fallback`
- `outputs/runtime/vibe-sessions/2026-04-11-full-host-mcu-audit`
- `outputs/runtime/vibe-sessions/2026-04-11-full-host-mcu-reaudit`

## 远程验证边界

当前 `/mnt/ros2_ws` 是挂载目录，本地禁止构建、运行 ROS、检查 topic/node/tf 或访问硬件。验证必须在远程机器人执行。

推荐远程验证入口：

```bash
cd ~/ros2_ws
colcon build --packages-select stm32_robot_bridge robot_bringup
source install/setup.bash
ros2 launch robot_bringup system.launch.py mode:=mapping mapping_source:=lidar base_mode:=real base_fusion_mode:=ekf
```

可选检查：

```bash
ros2 topic echo /odom
ros2 topic echo /odometry/filtered
ros2 topic echo /imu/data
ros2 topic echo /scan
```

## 未关闭风险

- 实机 IMU 轴向、符号、安装偏角仍需通过静止、手动旋转、原地转向三类测试确认。
- Nav2 参数需在真实场地根据底盘响应和雷达质量继续调低或调平滑。
- ESP01S 长按卡顿已做静态时序修正，仍需烧录 ESP01S 后实机确认 `/health.loop_max_gap_ms`、`uart_tx_timeouts` 和长按手感。
- PS2/ESP01S 人工遥控不再默认叠加绝对航向保持，但启用 IMU 实际偏航率优先的直行扶正，编码器偏航率仅作 fallback；针对固定左偏加入 `STRAIGHT_SIDE_TRIM=0.04` 静态补偿。闭环轮速控制仍保留，需刷入 STM32 后实机确认。
- 雷达串口固定为 `/dev/ttyUSB0` 依赖当前插线顺序；长期建议使用 udev 规则生成稳定别名。
