# 包职责矩阵

| 包 | 负责 | 不负责 | 主要输入 | 主要输出 |
| --- | --- | --- | --- | --- |
| `robot_interfaces` | Platform API 消息常量与结构 | 运行逻辑 | 无 | ROS interface |
| `robot_config` | 手写配置事实、校验、编译与 hash | 运行状态机 | canonical YAML | effective config、ROS params |
| `robot_control` | 来源仲裁、校验、lease、运动限制、enable/rearm 意图 | 串口、ACK、里程计、TF | `cmd_vel/*`、安全状态 | `ChassisCommand`、`ControlState` |
| `stm32_robot_bridge` | 串口、framing、v3 codec、HELLO/session/ACK、通信 fail-closed、协议到观测转换 | 里程计、协方差、运动学监督、Nav2 | `ChassisCommand`、UART | Observation、`ChassisState`、诊断 |
| `robot_state_estimation` | 样本顺序、MCU 时间映射、轮式里程计、IMU 质量、协方差、EKF 编排 | 串口、命令仲裁 | Observation | `wheel/odom`、`imu/data`、`odom`、TF |
| `robot_supervision` | 命令跟踪、轮一致性、非预期运动、降级/释放建议 | 串口、里程计积分、最终命令发布、Nav2 Goal 所有权 | 命令、Observation、IMU | `MotionSafetyState` |
| `robot_verification` | Fake Base、故障场景、graph/bag 契约工具 | 生产估计算法 | `ChassisCommand`、场景 | 与真实底盘相同的平台状态接口 |
| `robot_description` | URDF/Xacro、静态 frame 关系和机械事实 | 动态 TF、运行算法 | 硬件/标定配置 | `robot_description`、静态 TF |
| `robot_sensing` | 雷达驱动与 scan 规范化 | 底盘和状态估计 | 设备数据 | `scan` |
| `robot_bringup` | 模式选择、节点编排、参数注入、启动健康检查 | 业务算法和业务状态 | launch arguments、effective config | ROS runtime graph |

禁止依赖：`robot_control` 不依赖 Bridge；状态估计和监督不依赖 Bridge
内部 Python 类型；Bridge 不依赖里程计算法；bringup 不包含业务算法。
