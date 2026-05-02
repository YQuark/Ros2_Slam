# 上位机与下位机代码全面审计需求冻结

## 目标

对当前机器人上位机 ROS2 工作区与下位机 STM32/ESP01S 工程进行一次全面静态代码审计，并输出可交付的详细审计说明文档。

## 范围

- 上位机自研代码：
  - `src/stm32_robot_bridge`
  - `src/robot_bringup`
  - `launch_scripts`
- 上位机集成边界：
  - `src/ydlidar_ros2_driver`
  - `src/ros2_astra_camera`
  - `src/third_party`
- 下位机代码：
  - `/mnt/d/Document/Work/projects/Clion/Core`
  - `/mnt/d/Document/Work/projects/Clion/ESP01S`

## 约束

- 当前 `/mnt/ros2_ws` 是远程机器人工作区的 SMB 挂载，本地只允许阅读和编辑。
- 禁止在本地执行 `colcon build`、`cmake`、`make`、`ninja`、`ros2 launch`、`ros2 run`、topic/node/tf 检查和任何硬件相关程序。
- 禁止写入 `build/`、`install/`、`log/` 目录。
- 审计结论只能表述为静态审计结论，运行正确性需在远程机器人环境验证。

## 交付物

- `/mnt/d/document/projects/上位机与下位机代码全面审计说明.docx`
- 本工作区内保留 `$vibe` 需求、计划和执行回执，便于追溯审计依据。

## 验收标准

- 文档覆盖上位机、下位机 STM32、ESP01S、协议链路、启动脚本、传感器集成、配置和安全风险。
- 每个主要问题包含证据位置、影响、建议修复方向和优先级。
- 明确区分已静态确认的问题、需要远程机器人验证的问题和第三方代码边界风险。
