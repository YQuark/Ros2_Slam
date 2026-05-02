# 2026-04-09 航向/定位/底盘控制审计执行计划

## Internal Grade

L

原因：任务是多文件静态审计，需要跨仓库比对，但不需要并行改动或大规模实现。

## Execution Batches

### Batch 1

- 审计 `robot_bringup` 的 launch/config/RViz
- 审计 `stm32_robot_bridge` 的协议解码、odom/imu 发布、TF 行为
- 建立 `cmd_vel -> base -> odom/imu -> ekf -> slam/nav2` 链路图

### Batch 2

- 审计外部底盘仓库 `YQuark/SLAM_ROBOT`
- 核对串口协议文档与上位机桥接行为是否一致
- 查找与 `yaw_est`、`v_est/w_est`、控制语义相关的契约

### Batch 3

- 汇总问题并按严重度分级
- 区分“确定性缺陷”“配置空洞”“需远程验证风险”
- 产出远程验证命令模板与检查建议

## Ownership Map

- 根任务：当前会话
- 审计对象 1：`src/robot_bringup`
- 审计对象 2：`src/stm32_robot_bridge`
- 审计对象 3：`https://github.com/YQuark/SLAM_ROBOT`

## Verification Commands

当前目录是挂载环境，不能在本地执行，请在远程机器人通过 SSH 执行。

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup stm32_robot_bridge
source install/setup.bash
ros2 launch robot_bringup system.launch.py mode:=mapping mapping_source:=lidar base_mode:=real base_fusion_mode:=ekf
```

补充定位验证：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_bringup system.launch.py mode:=navigation map_file:=/home/robot/ros2_maps/latest.yaml base_mode:=real base_fusion_mode:=ekf
ros2 topic list
ros2 node list
ros2 topic echo /odom
ros2 topic echo /imu/data
ros2 topic echo /odometry/filtered
```

## Delivery Acceptance Plan

- 以文件证据证明问题所在
- 明确哪些结论已经足够确定，哪些只能作为远程验证假设
- 最终结论只宣称“完成静态审计”，不宣称“修复完成”

## Completion Language Rules

- 只有在证据充分时才使用“确定问题”
- 对未能本地运行验证的内容使用“高概率风险”或“需远程验证”

## Rollback Strategy

- 本次默认不改业务代码
- 若后续需要修复，采用最小必要修改，优先改桥接参数与配置，再考虑算法层

## Cleanup Expectations

- 写入 `outputs/runtime/vibe-sessions/2026-04-09-heading-audit/`
- 保留 skeleton、intent、phase、cleanup 收据
- 不产生临时构建文件

## Specialist Dispatch

- 无
