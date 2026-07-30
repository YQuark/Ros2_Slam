# 包职责矩阵

| 包 | 负责 | 明确不负责 | 主要输出 |
| --- | --- | --- | --- |
| `robot_interfaces` | Platform API 5 消息、Action 和常量 | 运行逻辑 | ROS interfaces |
| `robot_config` | canonical YAML、Draft 7 schema、跨参数校验、编译、hash、兼容门 | 算法状态和固件参数写入 | effective config、ROS/SLAM/Nav2/Lidar 参数、manifest |
| `robot_chassis_model` | LF/LR/RF/RR 布局与左右聚合 | ROS、串口 | 纯布局算法 |
| `robot_control` | Host 子来源仲裁、有限值/范围校验、lease、运动限制、Host rearm | 整车来源仲裁、物理许可、串口 | `HostMotionCommand`、`HostControlState` |
| `stm32_robot_bridge` | transport、framing、Upper v3、HELLO、wire session、ACK、超时、原始状态适配 | odom、TF、协方差、监督、导航 | Observation、链路/固件状态、diagnostics、私有 wire services |
| `robot_chassis_ops` | 将私有 wire service 包装为有证据语义的公共 Action | 电机控制和安全许可 | `chassis/operation` |
| `robot_state_estimation` | 样本顺序、MCU 时间映射、正式 wheel odom、IMU 质量、可选 EKF 编排、正式 odom/TF gate | 串口和命令仲裁 | `wheel/odom`、`imu/data`、`odom`、`odom->base_link` |
| `robot_supervision` | 轮间/跟踪/yaw/非预期运动一致性风险与 Host 建议 | 物理安全许可、里程积分 | `MotionSupervisionState` |
| `robot_navigation_guard` | 公共 Goal 准入、撤权取消、旧 UUID 终止和新 Goal generation | 路径规划、底盘控制 | `navigate_to_pose` proxy、`NavigationGuardState` |
| `robot_verification` | Semantic Fake Base、固件 emulator、故障场景和验证支撑 | 生产估计算法 | 与真实 provider 相同的 Platform API |
| `robot_description` | frame/机械静态关系 | 动态 TF 和算法 | robot description、静态 TF |
| `robot_sensing` | 雷达接入、scan 规范化和设备诊断 | 底盘、SLAM 状态 | `scan_raw`、`scan` |
| `robot_bringup` | 模式选择、节点编排和参数注入 | 业务算法与持久状态 | runtime graph |

跨包只通过公开 ROS interface 或纯 DTO/算法接口依赖。Bridge 不向状态估计泄漏
协议内部类型；`robot_control` 不依赖 Bridge；bringup 不实现业务状态机。
