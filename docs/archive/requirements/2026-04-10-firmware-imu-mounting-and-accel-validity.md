# Firmware IMU Mounting And Accel Validity Requirements

日期：2026-04-10

## Goal

在已知“主板平行于地面、MPU6050 模块垂直插在主板上”的真实安装前提下，重新审计 STM32 下位机的 IMU 安装轴映射、姿态输入语义和 `imu_accel_valid` 判定，避免在证据不足时误改轴向，同时收紧当前过宽的加速度可用性定义。

## Deliverable

- 明确当前固件中原始加速度、姿态融合输出、`yaw_est` 与 `imu_accel_valid` 的真实语义
- 明确当前轴映射是否被机械安装事实支持
- 如果证据充分，仅修改下位机中与 IMU 轴映射判定或 `imu_accel_valid` 判定直接相关的最小代码
- 输出风险说明与远程验证方法

## Constraints

- 不基于猜测翻转 IMU 轴映射
- 不修改上位机协议格式
- 不把原始加速度直接接入平移里程计
- 不在当前挂载环境本地编译或运行

## Acceptance Criteria

- 审计结论能区分“原始体坐标加速度”与“去重力线加速度”
- 审计结论能区分“航向可用”与“加速度可用”
- 若实施代码修改，修改范围只限于 IMU 安装映射和 `imu_accel_valid` 相关逻辑
- 所有结论均有源码证据支撑

## Product Acceptance Criteria

- `yaw_est` 继续作为下位机优先航向输出
- `imu_accel_valid` 不再仅由模长接近 `1g` 就轻易置真
- 静止时对加速度的判断能更符合当前真实安装语义

## Manual Spot Checks

- 静止时，导出的原始体坐标加速度主分量应主要落在车体竖直轴
- 静止时，若安装和映射正确，横向两个体轴加速度应明显小于竖直轴
- `imu_accel_valid=1` 时，应满足静态校准与运行期门限，而不仅是模长过关

## Completion Language Policy

- 未修改代码前，不允许声称“已修复”
- 未经远程编译和实机复测，不允许声称“验证通过”

## Delivery Truth Contract

- 本次交付只声称“完成了机械安装前提下的固件语义审计，并在证据充分时给出最小修正”
- 不声称“IMU 最终标定完成”

## Non-Goals

- 不重写姿态融合算法
- 不把加速度用于上位机平移积分
- 不扩展新的协议字段

## Inferred Assumptions

- 当前上位机 `/imu/data.linear_acceleration` 来自固件导出的原始体坐标加速度，而不是去重力线加速度
- 现有“车体竖直轴对应传感器 +X”假设与实测数据并不矛盾，暂不能仅凭安装描述推翻
