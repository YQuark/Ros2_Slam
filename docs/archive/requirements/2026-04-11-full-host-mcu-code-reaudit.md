# 上位机与下位机代码二次全面审计需求冻结

## 背景

用户在上一版审计后又修改了较多上位机和下位机代码，要求再次全面审计，并允许更新 `/mnt/d/document/projects/上位机与下位机代码全面审计说明.docx`。

## 目标

重新审计当前代码状态，识别已修复项、新增风险和残留风险，并覆盖更新既有 Word 审计说明。

## 范围

- 上位机：`src/stm32_robot_bridge`、`src/robot_bringup`、`launch_scripts`、`src/ydlidar_ros2_driver` 集成边界。
- 下位机：`/mnt/d/Document/Work/projects/Clion/Core`、`/mnt/d/Document/Work/projects/Clion/ESP01S`。
- 重点关注本轮改动涉及的速度量纲、里程计、状态扩展、IMU/EKF、串口探测、雷达串口排除、ESP Web 控制、模式切换和运维脚本。

## 约束

- 当前目录为远程机器人 SMB 挂载，本地只做静态审计和文档更新。
- 禁止本地构建、运行、ROS topic/node/tf 检查、硬件程序。
- 禁止写入 `build/`、`install/`、`log/`。

## 交付物

- 更新后的 `/mnt/d/document/projects/上位机与下位机代码全面审计说明.docx`。
- 本轮 `$vibe` 需求、计划、执行和清理回执。
