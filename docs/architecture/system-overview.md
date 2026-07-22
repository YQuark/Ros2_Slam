# 系统架构

运行链固定为 `Twist source → robot_control → ChassisCommand v3 → BridgeCore → upper v3 → STM32`。`robot_control` 是 `chassis/command` 的唯一发布者；Bridge 只发布 `wheel/observation`、`imu/observation` 与通信状态；`robot_state_estimation` 生成 `wheel/odom` 和 `imu/data`，`robot_localization` 独占 `odom → base_link` TF。

Bridge 的 ROS adapter 负责 topic、service、timer 和消息转换。串口线程只做有界 I/O，v3 codec 只做帧转换，`BridgeCore` 以不可变快照处理通信安全。`robot_supervision` 按四轮一致性、目标跟踪、wheel/gyro yaw 和非预期运动分级提出限速或 release；最终命令状态机仍由 `robot_control` 拥有。

Fake Base 与 Real Base 都实现同一底盘提供者契约：订阅 `ChassisCommand`，发布 `WheelObservation`、`ImuObservation`、`ChassisState`、`FirmwareInfo` 和 diagnostics。两者都不得直接发布 odometry 或 TF。
