# 上位机与下位机代码静态审计执行计划

日期: 2026-04-11
内部等级: L

## 执行结构

1. Skeleton check
   - 确认 ROS2 工作区、AGENTS 限制、代码位置、dirty worktree。
2. 上位机审计
   - 阅读 `stm32_robot_bridge` 协议、串口、里程计、IMU 发布代码。
   - 阅读 `robot_bringup` 的 base/system/ekf/sensor launch 与关键配置。
3. 下位机审计
   - 阅读 STM32 LinkProto、PC/ESP 串口、RobotControl、安全配置。
   - 阅读 ESP01S HTTP/UART 桥接代码。
4. 交叉契约审计
   - 对比 Q15 归一化、速度量纲、状态包长度、yaw/IMU/odom 语义。
5. 输出审计结论
   - 按严重度列出风险、证据、影响和建议。
6. Cleanup
   - 仅保留审计文档和 runtime 收据，不产生构建或运行产物。

## 验证策略

本地不执行编译、运行或 ROS 检查。需在远程机器人上验证:

```bash
cd ~/ros2_ws
colcon build --packages-select stm32_robot_bridge robot_bringup
source install/setup.bash
ros2 launch robot_bringup system.launch.py
```

可选检查:

```bash
ros2 topic list
ros2 node list
ros2 topic echo /odom
ros2 topic echo /imu/data
ros2 topic echo /cmd_vel
```

## 完成语言规则

不能写“已运行成功”或“验证通过”。只能说明“已完成静态审计”与“需在远程环境验证”。

## 回滚规则

本轮不修改功能代码，无功能回滚项。若需移除审计产物，可删除本计划、需求文档和对应 `outputs/runtime/vibe-sessions/2026-04-11-host-mcu-audit/`。
