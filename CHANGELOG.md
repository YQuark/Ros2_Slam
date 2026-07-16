# Changelog

本项目的重要变更记录在此文件中。

## [Unreleased]

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
