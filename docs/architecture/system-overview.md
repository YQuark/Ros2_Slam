# 当前系统架构

## 职权边界

Platform API 5 把 Host 意图、传输事实、物理控制和状态估计分开：

- `robot_control`：候选 `Twist` 的校验、优先级、lease、速度/加速度/jerk 限制和 Host rearm。
- `robot_supervision`：运动一致性风险、Host 降级倍率和 Host release 建议。
- `stm32_robot_bridge`：串口、framing、Upper v3 codec、HELLO、wire session、ACK、超时和状态适配。
- 固件：Host/PS2/ESP/Line/Debug 最终仲裁、物理运动许可、闭环控制和电机输出。
- `robot_state_estimation`：累计编码器里程计、IMU 字段质量、MCU 到 ROS 时间映射、可选融合、正式 odom 和动态 TF。
- `robot_sensing`：YDLidar 设备接入、scan 规范化和设备诊断。
- `robot_config`：手写配置、严格校验、编译产物和配置身份。
- `robot_bringup`：节点编排，不持有算法状态。
- `robot_verification`：Fake Base、PTY、Bag、HIL、标定和发布证据。

Bridge 不计算 odometry，不发布 TF，不执行 Host 子来源仲裁，也不作 Nav2 决策。
上位机 release 只撤回 Host 候选；整车是否物理停止必须由新鲜固件 STATUS 证明。

## 数据与控制链

```text
teleop/test/nav/research Twist
          |
          v
robot_control -- HostMotionCommand + HostControlState
          |
          +------------------------------+
          v                              v
stm32_robot_bridge                  Fake Base
          |                              |
          +---- common Platform API -----+
                     |
      WheelObservation / ImuObservation
      ChassisLinkState / FirmwareControlState / FirmwareInfo
                     |
      +--------------+------------------+
      v                                 v
robot_state_estimation            robot_supervision
  wheel/odom, imu/data               MotionSupervisionState
      |                                 |
      +-> optional internal EKF          +-> robot_control
      |
      v
formal_odometry -> /odom + odom->base_link
```

Fake Base 和真实 Bridge 是互斥 provider。两者发布相同 Platform API 状态，但都不得
发布 odometry 或 TF。

## 正式里程计与融合

默认 `fusion_mode: wheel`：`wheel_odometry` 生成 `wheel/odom`，
`formal_odometry` 在兼容门和当前 session 新鲜观测允许时转发为 `/odom` 并发布 TF。

显式 `fusion_mode: wheel_imu`：`robot_localization/ekf_node` 只发布内部
`odometry/filtered_internal`，仍由 `formal_odometry` 完成正式出口和 TF。EKF 目前是
实验路径；独立 Bag 对照、IMU 标定和降级证据通过前不得改成默认。

## 兼容门与发布状态

`platform_compatibility` 同时检查当前 wire session、HELLO 身份、协议/schema、
capability、硬件 revision、参数 CRC、轮布局和新鲜观测。真实平台任一事实缺失时，
正式 odom 和导航均 fail-closed；Fake Base 通过显式 `simulated` 身份进入 simulation
状态。

当前候选固件为 `bc472cc874e930aaed6eb8e7de73b41a2563dd85`，但 tested commit、
参数 CRC、硬件 revision、UART HIL、标定和实车报告尚未锁定，
`release_compatible` 仍为 `false`。
