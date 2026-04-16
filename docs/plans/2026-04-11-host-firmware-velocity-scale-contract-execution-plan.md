# Host Firmware Velocity Scale Contract Execution Plan

日期：2026-04-11

## Internal Grade

`L`

原因：

- 问题集中在桥接层协议尺度
- 下位机归一化语义合理，不应优先改固件协议

## Steps

1. 审计上位机 Q15 编码与反馈反缩放
2. 审计下位机 Q15 接收、反馈归一化和物理尺度来源
3. 计算固件归一化物理尺度
4. 修改上位机默认桥接尺度
5. 输出远程验证步骤

## Ownership Boundaries

- `/mnt/ros2_ws/src/stm32_robot_bridge/stm32_robot_bridge/bridge_node.py`
- `/mnt/ros2_ws/src/stm32_robot_bridge/launch/stm32_bridge.launch.py`
- `/mnt/ros2_ws/src/robot_bringup/launch/base.launch.py`
- `/mnt/ros2_ws/src/robot_bringup/launch/system.launch.py`

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
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}, angular: {z: 0.0}}" -r 5
ros2 topic echo /odom
ros2 topic echo /imu/data
```

## Delivery Acceptance Plan

- `/cmd_vel` 小速度命令对应合理的 `/odom.twist`
- 角速度命令不再被固件按错误尺度放大
- Nav2 限速仍保持在安全范围

## Completion Language Rules

- 未经远程环境验证，不允许使用“验证通过”

## Rollback Rules

- 若实测速度仍偏差大，保留协议尺度一致性，继续做机械速度标定，不回退到错误默认值

## Phase Cleanup

- 保留 requirement / plan / runtime receipt
- 不新增临时测试文件
