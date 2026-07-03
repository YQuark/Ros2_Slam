# stm32_robot_bridge

`stm32_robot_bridge` 是当前工程的 STM32 底盘串口桥接包。它把 ROS2 `/cmd_vel/driver` 转成下位机 v2 上位机协议，并把下位机主动上报的 STATUS 转成 `/wheel/odom`、电池与电流状态。

## 包职责

- 订阅 `/cmd_vel/driver`，发送 `SET_VELOCITY` (`0x01`)。
- 被动接收下位机主动上报的 `STATUS` (`0x81`)，不发送状态请求。
- 发布 `/wheel/odom`、`/battery_state`、`/motor/left_current`、`/motor/right_current`、`/chassis/status`。
- 不默认发布 `odom -> base_link` TF；标准 `/odom` 与 TF 由 `robot_state_estimation` 统一发布。
- 提供 `/chassis/estop` (`std_srvs/SetBool`) 显式触发或解除急停。

当前下位机 v2 STATUS 不包含 IMU 字段，本包不会发布伪造 IMU 数据。

## 当前协议

权威来源为下位机本地 `D:\Document\Work\projects\F407_V2.0` 当前 `main` 分支：

- `App/protocol/upper_protocol.h`
- `App/protocol/upper_protocol.c`
- `App/protocol/upper_uart.c`

关键约定：

- 串口：Raspberry Pi GPIO UART，默认 `/dev/serial0`，115200 8N1。
- 帧格式：`0xA5 0x5A + length + cmd + payload + crc8`。
- `length` 是 `cmd + payload` 的字节数，最大 65。
- CRC8 覆盖 `length + cmd + payload`，初值 0，多项式步骤为最高位为 1 时异或 `0x5E`。
- `SET_VELOCITY` payload 固定 10B：`float linear_x + float angular_z + uint8 enable + uint8 mode`，上位机发送 `mode=2`。
- `STATUS` payload 固定 64B，version 必须为 2。

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
- `/cmd_vel/driver` 有效期内下位机 `control_source=1`。
- `/cmd_vel/driver` 超时后 bridge 只发送一次 `enable=0` 释放上位机控制源。
- `/chassis/estop` 触发后 STATUS bit0 置位，解除需显式调用服务发送 `false`。

## 相关文档

- [串口桥接协议](../../docs/04-串口桥接协议.md)
- [底盘与串口桥接](../../docs/06-底盘与串口桥接.md)
- [系统架构](../../docs/02-系统架构.md)
