# Changelog

本项目的重要变更记录在此文件中。

## [Unreleased]

### v0.6.0-rc1 implementation

- ROS Platform API 升级为 v5，冻结 Host 候选、Bridge 协议适配、固件整车仲裁、安全许可和电机输出的职权边界；Upper v3 字节协议不变。
- Bridge/robot_control 实现双层 rearm 状态机、会话绑定兼容门和单调时间租约；相同 Host 目标刷新租约但不推进 wire sequence。
- 里程计统一支持 M2+M3 与四驱布局，按 session/reset/mask 重建基线，并将累计 Pose 协方差与逐样本 Twist 协方差分离。
- Motion Supervision 明确只释放 Host 候选；Navigation Guard 改为公开 Action 代理，Chassis Ops 收口为单一 Action，内部 wire 服务私有化。
- 新增统一标定包身份、Semantic Fake 结构门和可复用 Upper-v3 PTY firmware emulator；软件构建和测试通过，真实 HIL/标定/实车报告仍为 `NOT_RUN`。

### v0.5.0-rc1 implementation

- ROS Platform API 升级为 v4：`HostMotionCommand` 只表示 Host 候选，并拆分 wire link、firmware control、Host control 和 motion supervision 状态。Upper Protocol v3 字节契约保持不变。
- 新增统一轮布局模型，里程计、协方差和运动监督按 `motor_enabled_mask & speed_valid_mask & ~anomaly_mask` 消费默认 M2+M3 两驱观测。
- Bridge 只发布新 STATUS/IMU 样本，实现健康 STATUS 后 fresh disable + 精确 `ACK_APPLIED` 的 wire rearm，并将本地 lease/watchdog/limiter 计时改为单调时钟。
- 新增 `robot_navigation_guard`、`robot_chassis_ops` 和参数 CRC/硬件身份兼容门；旧 Nav2 Goal 在故障恢复后不会自动复用。
- Semantic Fake Base 默认模拟 M2+M3，PTY 测试覆盖 v3 session/ACK/watchdog，CI 与下位机 `366a038` 黄金向量交叉校验。
- 发布门已迁移到 v0.5.0-rc1；真实 UART HIL、参数 CRC、实车与最终标定仍为 `NOT_RUN`，因此保持 fail-closed。

### v0.4.0 implementation

- Platform API 与 upper wire protocol 升级为 v3；运行时移除 v2 fallback，引入 session、sequence、ACK、HELLO 身份与能力门。
- 新增 `robot_config` 单一事实源编译器、跨参数校验、effective config 与 SHA-256 运行清单。
- bridge 改为 ROS adapter、纯 `BridgeCore`、有界独立串口 I/O、显式 QoS 和 fail-closed 状态快照。
- 采用 encoder count 增量 SE(2) 里程计和协方差传播；IMU 改为字段级质量与 affine MCU 时钟估计。
- MotionLimiter 同时约束速度/加速度/jerk；MotionSupervisor 按一致性逐级提高协方差、降速并 release。
- SLAM profile 改为 base + overlay，实验升级为 LHS 粗搜索、交互筛选和独立验证集；fallback 重命名为未认证 example。
- 验证与发布证据迁移到 `verification` 机器格式。v3 firmware、HIL、实车和标定仍为 NOT_RUN/provisional，因此 v0.4.0 发布保持阻塞。

### Added

- 建立上位机 v0.3.0 的固件兼容矩阵、统一验证入口和版本化验收报告。
- 新增 `robot_interfaces/ChassisCommand` 与 `ChassisState`，提供显式 enable、来源和 sequence。
- mux 拒绝 NaN、Inf 和异常绝对值，按来源清除失效命令并记录拒绝计数。
- bridge 增加硬速度包络、命令年龄检查、STATUS 驱动许可和通信状态机。
- 串口启动改为非阻塞三次 RELEASE，写入支持分段短写并记录 tx/rx/reconnect 统计。
- 同步固定固件提交的 upper v2 黄金 fixture；发布 `ChassisState` 和五组件标准 ROS diagnostics。
- wheel odom 改为 STATUS 样本驱动，拒绝重复时间与过大通信间隙的旧速度补积分。
- IMU 使用 MCU 时间与 sample_count，拒绝质量异常/重复样本并发布显式协方差。
- 新增唯一版本化 `robot_calibration.yaml`；mux 使用统一加速度/jerk 软整形器平滑来源切换。
- wheel odom 使用速度、转向、采样间隔和左右轮分歧计算动态协方差；默认 EKF 仅融合 wheel `vx` 与 IMU `gyro_z`。
- 新增 7 类版本化 rosbag 基准数据集目录、可复现录制清单和严格 metadata 验收工具。
- 新增 SLAM 六参数单因素矩阵生成器、结果模板和九指标严格验收门。
- 新增 Nav2 底盘动态前置门、九参数矩阵与 10 点×5 次重复导航验收器。
- CI 新增 ruff/black/mypy/yamllint/shellcheck、分模块覆盖率门和 PTY 伪 STM32 串口故障测试。
- 新增 HIL/实车 fail-closed 报告、v0.3.0 发布候选升级/回滚说明和机器发布门。

### Changed

- 更正固件 UPPER 控制源超时契约：当前 beta4 为 200 ms；500 ms 宏已废弃。
- Platform API 升级至 2；`/chassis/command` 成为默认底盘命令，裸 Twist 仅保留显式兼容开关。
- mux 在 active→idle 时仅发布一次释放，idle 状态不再持续占用 UPPER。
- ESTOP/fault-stop 恢复后必须观察到 disable→enable 边沿，禁止旧命令自动复动。
- 实车默认由 EKF 独占发布 `odom -> base_link`，bridge 的 TF 发布保持关闭。
- 所有自有 ROS package 版本统一升级为 `0.3.0`。

## [0.2.0] - 2026-07-11

- 对齐 STM32 upper v2 协议并接入 STATUS、IMU_STATUS 与 DIAGNOSTIC。
- 建立 ROS 2 Humble 构建、pytest、覆盖率和仓库边界 CI。
