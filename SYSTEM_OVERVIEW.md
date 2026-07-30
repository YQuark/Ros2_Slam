# 系统架构索引

当前实现基线为 Platform API 5、Upper Protocol v3、候选版本
`v0.6.0-rc2`。本文件只做入口索引，代码级事实以消息定义、配置事实源和下列
权威文档为准。

## 推荐阅读顺序

1. [当前系统架构](./docs/architecture/system-overview.md)
2. [包职责矩阵](./docs/architecture/package_responsibility_matrix.md)
3. [Topic 所有权](./docs/architecture/topic_ownership_matrix.md)
4. [TF 所有权](./docs/architecture/tf-ownership.md)
5. [配置单一事实源](./docs/architecture/configuration.md)
6. [Host 命令契约](./docs/contracts/host-command-contract.md)
7. [观测契约](./docs/contracts/observation-contract.md)
8. [双层 rearm 契约](./docs/contracts/rearm-contract.md)
9. [Upper Protocol v3](./docs/interfaces/upper-protocol-v3.md)
10. [rc2 状态与发布门](./docs/releases/v0.6.0-rc2.md)

## 当前主链

```text
Twist candidates
  -> robot_control
  -> HostMotionCommand
  -> Real Bridge / Fake Base
  -> Upper v3 / simulated provider

WheelObservation -> wheel_odometry -> wheel/odom --------+
ImuObservation   -> imu_adapter    -> imu/data             |
                                                          +-> formal_odometry
wheel_imu only: wheel/odom + imu/data -> internal EKF -----+   -> /odom + odom->base_link
```

`robot_control` 只拥有 Host 子来源选择；固件拥有最终来源仲裁、物理运动许可和
电机输出。Bridge 不拥有里程计、TF、监督或导航。无论 wheel-only 还是
wheel+IMU，正式 `/odom` 与 `odom -> base_link` 的唯一发布者都是
`formal_odometry`。

运维入口为 `bin/robot`，工程编排入口为
`src/robot_bringup/launch/system.launch.py`。真实底盘必须使用编译后的 effective
config；当前参数 CRC、UART HIL、正式标定和实车证据尚未完成，因此不能宣称真实
导航已经发布兼容。
