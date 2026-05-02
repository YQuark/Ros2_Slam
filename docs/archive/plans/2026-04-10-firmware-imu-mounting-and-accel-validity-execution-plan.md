# Firmware IMU Mounting And Accel Validity Execution Plan

日期：2026-04-10

## Internal Grade

`L`

原因：

- 问题集中在单一固件工程的 IMU 数据语义
- 需要串行完成：审计、收敛根因、最小修正、远程验证说明
- 当前证据链比并行试错更重要

## Steps

1. 冻结真实安装前提与交付边界
2. 核对原始加速度、姿态融合、`yaw_est`、`imu_accel_valid` 的源码链路
3. 判断当前轴映射是否与机械安装和实测一致
4. 仅在证据充分时，最小修改 `imu_accel_valid` 或轴映射相关逻辑
5. 给出远程验证命令与剩余风险

## Ownership Boundaries

- `/mnt/d/Document/Work/projects/Clion/Core/Inc/Drivers/robot_config.h`
- `/mnt/d/Document/Work/projects/Clion/Core/Src/Drivers/mpu6050.c`
- `/mnt/d/Document/Work/projects/Clion/Core/Src/Drivers/robot_control.c`
- `/mnt/d/Document/Work/projects/Clion/docs/serial_protocols.md`

## Verification Commands

当前目录是挂载环境，不能在本地执行，请在远程机器人通过 SSH 执行。

```bash
cd ~/ros2_ws
colcon build --packages-select stm32_robot_bridge robot_bringup
source install/setup.bash
ros2 launch stm32_robot_bridge stm32_bridge.launch.py port:=/dev/ttyUSB0 publish_tf:=false publish_imu:=true use_status_yaw:=true
```

可选检查：

```bash
ros2 topic echo /imu/data
ros2 topic echo /odom
```

## Delivery Acceptance Plan

- 明确当前 `/imu/data.linear_acceleration` 的真实来源
- 明确是否应修改轴映射，还是只应收紧 `imu_accel_valid`
- 若有改动，保证改动只触及确定性问题点

## Completion Language Rules

- 未经远程环境验证，不允许使用“验证通过”
- 如果没有修改固件代码，必须明确说明“仅完成审计”

## Rollback Rules

- 若修改后的加速度判定导致 `imu_accel_valid` 长期为 `0`，优先回退判定门限，不回退航向链路
- 若轴映射证据不足，不得强行改动映射

## Phase Cleanup

- 留下 requirement / plan / runtime receipt
- 不新增临时固件测试文件
