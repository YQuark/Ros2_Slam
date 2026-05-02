# stm32_robot_bridge

`stm32_robot_bridge` 是当前工程的底盘串口桥接包，负责把上位机运动命令转换为下位机串口协议，并把下位机状态反馈转换成 ROS2 话题与 TF。

## 包职责

- 订阅 `/cmd_vel`，发送 `SET_VELOCITY` (0x01)
- 轮询 `STATUS` (0x81)，发布 `/odom`
- 按需发布 `/imu/data`、`/battery_state`、`/motor/left_current`、`/motor/right_current`、`/chassis/status`
- 在未启用 EKF 时发布 `odom -> base_link` TF

## 在工程中的位置

- 普通运行优先使用 `/home/robot/ros2_ws/launch_scripts/robot.sh`
- 包级 launch 入口：`launch/stm32_bridge.launch.py`
- 工程编排通过 `robot_bringup/launch/base.launch.py` 引入本包

## 当前协议假设

- 串口默认参数：`115200 8N1`
- 协议帧格式：`0xA5 0x5A + length(1B) + cmd(1B) + payload + checksum8(1B)`
- `SET_VELOCITY` (0x01)：`float linear_x + float angular_z + uint8 enable + uint8 mode`（10 字节）
- `STATUS` (0x81)：47 字节，包含轮速、编码器、电池、电流、IMU 原始数据、错误标志、控制模式
- `/odom.twist` 默认使用 STATUS 中的轮速计算
- 当前统一默认时序：`cmd_timeout=0.25s`、`drive_keepalive_sec=0.10s`

## 推荐启动方式

单独诊断底盘：

```bash
cd /home/robot/ros2_ws/launch_scripts
./robot.sh base
./robot.sh base --port /dev/ttyUSB1
```

主业务正常运行时，不单独操作本包，而是通过：

```bash
./robot.sh mapping lidar --real-base --ekf-base
./robot.sh navigation --real-base --ekf-base
```

## 直接包级调试

只有在二次开发或诊断桥接内部行为时，才直接调用包级 launch：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/ros2_ws/install/setup.bash
ros2 launch stm32_robot_bridge stm32_bridge.launch.py port:=auto baudrate:=115200
ros2 launch stm32_robot_bridge stm32_bridge.launch.py port:=/dev/ttyUSB1 status_log_interval_sec:=1.0 cmd_log_interval_sec:=1.0
```

默认正式导航流程已经把日志频率收得更低，避免长期运行时输出过密。

## 相关文档

- [底盘与串口桥接](../../docs/06-底盘与串口桥接.md)
- [系统架构](../../docs/02-系统架构.md)
