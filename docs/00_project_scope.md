# 项目范围

本分支目标是把 `YQuark/Ros2_Slam` 收口为 Raspberry Pi 4B 上运行的 ROS2 Humble 上位机工作区。

当前阶段只把主链路推进到：

- YDLIDAR X2 通过 `/dev/ydlidar` 发布 `/scan`
- `launch_scripts/robot.sh` 作为正式入口
- 支持 `--no-rviz` 无头运行
- 为后续 STM32 bridge、SLAM、Nav2 分阶段迁移保留目录和配置边界

硬件真值：

- 上位机：Raspberry Pi 4B
- 系统：Ubuntu 22.04 64-bit
- ROS2：Humble
- 雷达：YDLIDAR X2
- 底盘：STM32F407 两轮差速，下位机协议以当前下位机仓库代码为准

未实测参数不得写成确定值。轮径、轮距、编码器方向、电机方向、雷达 yaw、IMU 坐标系和速度上限都必须通过现场测试确认。
