# Host Firmware Velocity Scale Contract Requirements

日期：2026-04-11

## Goal

修复上位机 ROS 物理速度与 STM32 固件 Q15 归一化速度之间的尺度契约不一致问题，确保 `/cmd_vel` 下发、固件反馈 `v_est/w_est`、`/odom` 和 EKF 使用同一物理尺度。

## Deliverable

- 上位机桥接默认 `max_linear/max_angular` 与固件归一化底盘能力一致
- 不修改 STM32 协议格式
- 保持 Nav2/控制器使用独立低速限制保证运行安全

## Constraints

- 不修改下位机协议字段
- 不修改 topic 名称
- 不在当前挂载环境本地运行 ROS 或构建
- 不把导航速度限制和协议尺度混为一谈

## Acceptance Criteria

- 桥接节点 Q15 编码使用固件实际归一化物理尺度
- 桥接节点 Q15 反馈反缩放使用相同尺度
- `stm32_bridge.launch.py`、`base.launch.py`、`system.launch.py` 默认值一致
- Nav2 速度限制仍保持小于协议尺度

## Product Acceptance Criteria

- ROS 侧 `/odom.twist` 与下位机估计速度尺度一致
- `/cmd_vel` 的物理速度不再被固件放大解释
- 建图和导航的运动估计尺度更可信

## Manual Spot Checks

- 发布小速度 `/cmd_vel` 后，`/odom.twist.twist.linear.x` 应接近实际目标速度
- 原地小角速度命令不应被解释为过大的旋转
- Nav2 仍使用 `max_vel_x/max_vel_theta` 控制实际导航速度

## Completion Language Policy

- 未经远程环境验证，不允许声称“验证通过”

## Delivery Truth Contract

- 本次只声称“上位机桥接默认协议尺度已与固件归一化尺度对齐”
- 不声称机械速度标定已经最终完成

## Non-Goals

- 不重标定电机
- 不重写固件速度控制
- 不提高 Nav2 默认运行速度

## Inferred Assumptions

- 固件 Q15 `1.0` 对应 `MAX_CPS`
- 固件参数为 `MAX_CPS=13800`、`DIFF_WHEEL_RADIUS_M=0.0325`、`ENCODER_COUNTS_PER_REV=2340`、`DIFF_TRACK_WIDTH_M=0.125`
