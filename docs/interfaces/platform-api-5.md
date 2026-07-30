# Platform API 5

Platform API 5 是 ROS 上位机的职责和语义契约；Upper protocol v3
是 Bridge 与 STM32 之间的字节协议。两者版本独立：API 5 当前只
支持 wire v3，但 API 升级不意味着线协议必然改变。

## 权威来源

| 事实 | 来源 |
| --- | --- |
| API 版本 | `PLATFORM_API_VERSION` 和 `src/robot_config/config/platform.yaml` |
| ROS 字段 | `src/robot_interfaces/msg` 和 `action` |
| 运行参数 | `src/robot_config/config` 的编译产物 |
| 固件候选与发布兼容 | `compatibility/firmware.yaml` |
| 线协议 | [`upper-protocol-v3.md`](./upper-protocol-v3.md) |

## 主链路

```text
/cmd_vel/{teleop,test,nav,research}
  -> robot_control
  -> /chassis/host_motion_command
  -> stm32_robot_bridge
  -> Upper v3 / STM32 最终物理仲裁

Upper v3 STATUS / IMU_STATUS
  -> /wheel/observation, /imu/observation
  -> robot_state_estimation
  -> /wheel/odom, /imu/data
  -> [可选 wheel_imu: /odometry/filtered_internal]
  -> formal_odometry
  -> /odom + odom -> base_link
```

Bridge 不发布 odom/TF，不做导航决策，也不拥有整车最终物理许可。
`formal_odometry` 是唯一 `/odom` 和 `odom -> base_link` 发布者；
不论 wheel-only 还是实验 `wheel_imu`，外部所有权都不变。

## 公开接口

| 接口 | 写者 | 语义 |
| --- | --- | --- |
| `/cmd_vel/*` | teleop/test/Nav2/research | Host 候选意图，不是电机许可 |
| `/chassis/host_motion_command` | `robot_control` | 仲裁后的 Host 候选，带 epoch/sequence/enable |
| `/chassis/host_control_state` | `robot_control` | Host rearm、新鲜度和 DDS 计数器 |
| `/chassis/link_state` | base provider | 线会话、ACK、wire rearm 和链路时效 |
| `/chassis/firmware_control_state` | base provider | 固件实际控制源、ESTOP 和 fault 事实 |
| `/chassis/firmware_info` | base provider | commit、capability、hardware revision 和 parameter CRC |
| `/wheel/observation` | base provider | 传输会话内的轮系累计计数、轮速和质量位 |
| `/imu/observation` | base provider | Upper v3 IMU 观测和字段级质量 |
| `/motion/supervision_state` | `robot_supervision` | 运动一致性风险，不是滑移概率 |
| `/platform/compatibility_state` | compatibility gate | 正式 odom/导航许可 |
| `/wheel/odom` | wheel estimator | 未经外部兼容门的轮式估计 |
| `/imu/data` | IMU adapter | 单位、时间和质量转换后的 ROS IMU |
| `/odom` | `formal_odometry` | 经兼容/时效门的正式里程计 |
| `/scan` | scan normalizer | 归一化后的正式雷达扫描 |

公开 `NavigateToPose` 服务由 `robot_navigation_guard` 提供，后端是
`nav2/navigate_to_pose`。`ChassisOperation` 只承载 ESTOP、clear fault、
line control 和 GET_INFO，结果区分 queued/applied/rejected/timeout/cancelled。

## 时间、队列与会话

- 候选和 Host 命令使用 Reliable + KeepLast(1) + Volatile。
- Host 命令默认周期 50 ms，deadline 100 ms，lifespan 120 ms，来源
  lease 250 ms。
- ROS `header.stamp` 只用于 DDS/队列入口新鲜度；已接受命令的安全租约
  使用 monotonic 时钟，不受 `/clock` 回跳影响。
- Bridge 默认 150 ms 命令/ACK 超时、250 ms STATUS 超时；
  固件 UPPER watchdog 基线为 200 ms。
- duplicate 幂等忽略且不刷新租约；旧 session 和乱序不得改变
  当前合法目标。当前活动 epoch 的完整性错误会 release 并要求 rearm。

详细规则见 [Host 命令契约](../contracts/host-command-contract.md)、
[双层 rearm](../contracts/rearm-contract.md) 和
[时序预算](../contracts/timing_budget.md)。

## 观测与降级

Upper v3 没有独立 encoder-count-valid mask。API 5 的 count eligibility 是
`motor_enabled_mask & ~encoder_anomaly_mask`，speed eligibility 另外要求
`speed_valid_mask`。单侧只剩一轮时可降级积分并放大协方差；整侧无有效
编码器时不积分 pose。短时异常不提交 accepted baseline，session/reset/
长间隔则显式重建 baseline。

## 当前发布状态

`v0.6.0-rc2` 尚不是发布兼容状态。上位机 release candidate/tested
commit、固件 tested/compatible commit、parameter CRC 和 hardware revision
都尚未写入兼容清单；UART HIL、标定、车辆和长稳报告仍为
pending/NOT_RUN。任一文档不得把固件候选 `bc472cc...` 描述为
`compatible_commit`。
