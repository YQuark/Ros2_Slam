# 系统架构

运行链固定为 `Twist source → robot_control → ChassisCommand v3 → BridgeCore → upper v3 → STM32`。`robot_control` 是 `chassis/command` 的唯一发布者；bridge 只发布原始 `wheel/odom` 和 `imu/data`；`robot_localization` 独占 `odom → base_link` TF。

bridge 的 ROS adapter 负责 topic、service、timer 和消息转换。串口线程只做有界 I/O，v3 codec 只做帧转换，`BridgeCore` 以不可变快照处理安全状态。MotionSupervisor 按四轮一致性、目标跟踪、wheel/gyro yaw 和非预期运动分级执行 covariance、降速和 release。
