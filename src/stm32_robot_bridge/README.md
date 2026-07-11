# stm32_robot_bridge

`stm32_robot_bridge` 是当前工程的 STM32 底盘串口桥接包。它把 ROS2 `/cmd_vel/driver` 转成下位机 v2 上位机协议，并把下位机主动上报的 STATUS/IMU_STATUS 转成 `/wheel/odom`、电池、电流与 IMU 状态。

## 包职责

- 订阅 `/cmd_vel/driver`，发送 `SET_VELOCITY` (`0x01`)。
- 被动接收下位机主动上报的 `STATUS` (`0x81`) 和 `IMU_STATUS` (`0x83`)，不发送状态请求。
- 发布 `/wheel/odom`、`/battery_state`、`/motor/left_current`、`/motor/right_current`、`/chassis/status`、`/imu/data`。
- 不默认发布 `odom -> base_link` TF；标准 `/odom` 与 TF 由 `robot_state_estimation` 统一发布。
- 提供 `/chassis/estop` (`std_srvs/SetBool`) 显式触发急停；`false` 会被 bridge 拒绝，解除只能通过本地 USART1 控制台执行。

IMU 只来自独立 `IMU_STATUS` 帧；本包不会发布伪造 IMU 数据，也不自动接入 EKF。

## 当前协议

权威来源为下位机仓库 [Yaoser-x/SlamRobot_Chassis_Control](https://github.com/Yaoser-x/SlamRobot_Chassis_Control) 当前 `main` 分支：

- `Domain/protocol/upper_protocol.h`
- `Domain/protocol/upper_protocol.c`
- `Service/communication/upper_uart_service.c`

关键约定：

- 串口：Raspberry Pi GPIO UART，默认 `/dev/serial0`，115200 8N1。
- 帧格式：`0xA5 0x5A + length + cmd + payload + crc8`。
- `length` 是 `cmd + payload` 的字节数，最大 100。
- CRC8 覆盖 `length + cmd + payload`，初值 0，使用与下位机 `UpperProtocol_Checksum8()` 完全一致的 256 项查表。
- `SET_VELOCITY` payload 固定 10B：`float linear_x + float angular_z + uint8 enable + uint8 mode`，上位机发送 `mode=2`。
- `STATUS` payload 固定 65B，`IMU_STATUS` payload 固定 99B，version 必须为 2。

## 运行入口

优先使用统一入口：

```bash
cd /home/robot/ros2_ws/launch_scripts
./robot.sh base
./robot.sh mapping lidar --manual --real-base
./robot.sh navigation --real-base --map classroom_v1
```

包级调试：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/ros2_ws/install/setup.bash
ros2 launch stm32_robot_bridge stm32_bridge.launch.py port:=/dev/serial0 status_log_interval_sec:=1.0 cmd_log_interval_sec:=1.0
```

如临时使用 USB-UART，可显式覆盖 `port`，但默认路径不做自动 USB 扫描。

## 验证

```bash
colcon test --packages-select stm32_robot_bridge --event-handlers console_direct+
./launch_scripts/check_chassis.sh
./launch_scripts/test_base_cmd.sh --base-port /dev/serial0 --rotate-only
```

实机验收重点：

- 1 秒内收到 v2 STATUS。
- `/wheel/odom` 约 20Hz 更新。
- BMI270 在线时 `/imu/data` 可读取到 `sensor_msgs/Imu`。
- `/cmd_vel/driver` 有效期内下位机 `control_source=1`。
- `/cmd_vel/driver` 超时后 bridge 只发送一次 `enable=0` 释放上位机控制源。
- `/chassis/estop` 触发后 STATUS bit0 置位；服务的 `false` 请求会返回失败，解除需使用本地 USART1 控制台 `estop 0`。

## 相关文档

- [串口桥接协议](../../docs/04-串口桥接协议.md)
- [底盘与串口桥接](../../docs/06-底盘与串口桥接.md)
- [系统架构](../../docs/02-系统架构.md)
