# 上位机与下位机代码静态审计需求

日期: 2026-04-11

## 目标

对 ROS2 上位机代码和 STM32/ESP01S 下位机代码做静态审计，找出影响底盘控制、里程计、串口协议、传感器融合和运行安全的高风险问题。

## 范围

- 上位机: `/mnt/ros2_ws/src/stm32_robot_bridge`, `/mnt/ros2_ws/src/robot_bringup`, 相关 `launch_scripts`
- 下位机: `/mnt/d/Document/Work/projects/Clion/Core`, `/mnt/d/Document/Work/projects/Clion/ESP01S`
- 重点链路: `/cmd_vel -> stm32_robot_bridge -> STM32 LinkProto -> RobotControl -> status -> /odom /imu/data /tf`

## 约束

- 当前 `/mnt/ros2_ws` 是远程机器人工作区的 SMB 挂载目录。
- 本地只允许阅读代码、静态分析和新增审计文档。
- 禁止本地执行 `colcon build`、`cmake`、`make`、`ninja`、`ros2 launch/run`、硬件程序、topic/node/tf 检查。
- 禁止写入 `build/`, `install/`, `log/`。

## 验收标准

- 输出按严重度排序的静态审计结论。
- 每个主要结论包含文件和行号依据。
- 明确说明未做本地运行验证，需在远程机器人验证。
- 附远程验证命令模板。

## 非目标

- 不修改设备参数、topic、节点名、接口。
- 不修复代码，除非用户后续明确要求。
- 不对第三方 SDK 做全面源码审计。
