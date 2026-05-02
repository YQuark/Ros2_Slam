# 上位机与下位机帧时序静态审计需求冻结

## 背景

用户要求在现有导航、底盘桥接和文档统一工作基础上，再做一轮专项静态审计，重点确认：

- 上位机控制帧与状态帧的时序是否合理
- 上位机帧率与下位机控制周期、链路超时、保活节拍是否匹配
- 日志、返回、文档和代码中的时序语义是否一致
- `Ros2_Slam` 与 `Clion` 底盘代码之间的协议和运行假设是否清晰且可维护

## 目标

对当前工作区中的上位机 `Ros2_Slam` 与下位机 `Clion` 做一次以帧时序和控制链路为核心的完全静态审计，识别确定缺陷、高概率风险、时序漂移和文档不一致项；对能静态修复的问题直接修复。

## 范围

- 上位机：
  - `launch_scripts`
  - `src/stm32_robot_bridge`
  - `src/robot_bringup`
  - 已有主文档中与底盘链路、导航链路、运维入口相关的部分
- 下位机：
  - `/mnt/d/Document/Work/projects/Clion/Core`
  - `/mnt/d/Document/Work/projects/Clion/ESP01S`
  - `/mnt/d/Document/Work/projects/Clion/README.md`
  - `/mnt/d/Document/Work/projects/Clion/docs/serial_protocols.md`

## 重点审计问题

1. 上位机 `control_hz`、`status_hz`、`cmd_timeout`、`drive_keepalive_sec` 是否与下位机 `CTRL_PERIOD_MS`、`CMD_TIMEOUT_MS`、`LINK_ACTIVE_TIMEOUT_MS`、ESP `CONTROL_REFRESH_MS` / `CMD_IDLE_STOP_MS` 形成合理时序窗口。
2. `GET_STATUS`、`SET_DRIVE`、`SET_MODE` 的发送、接收、ACK/NACK、队列和恢复逻辑是否可能造成控制周期抖动、状态滞后或误判。
3. 下位机主循环、控制 pending/backlog 机制、UART 异步发送队列与上位机轮询频率是否在静态上匹配。
4. 上位机日志、返回值、默认参数、文档说明是否与实际时序语义一致。
5. 真底盘与假底盘的时序默认值是否足够接近，避免联调时出现行为漂移。

## 约束

- 本轮只做静态审计，不做 ROS2 运行态验证，不做硬件联机验证。
- 不执行实机命令、话题回放、串口真实通信或固件烧录。
- 不回退用户已有改动，不重写历史需求/计划文档。
- 若发现可以直接静态修复且风险明确的问题，应直接修改代码和文档。

## 验收标准

- 给出一张清晰的上下位机时序契约表。
- 明确区分：
  - 已静态证实正确的链路
  - 已静态证实存在的问题
  - 只能通过运行态确认的残余风险
- 所有被修改的默认值、日志和文档口径保持一致。
- 最终结论必须附带代码定位和静态验证结果，不允许无证据宣称“已完全修复”。

## 非目标

- 本轮不做导航算法替换。
- 本轮不做 EKF 实机调参。
- 本轮不做固件架构重写。
- 本轮不以运行结果替代静态证据。
