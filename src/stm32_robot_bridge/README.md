# stm32_robot_bridge

ROS2 bridge for `YQuark/STM32_Robot` serial protocol.

## Features
- Subscribe `cmd_vel` (`geometry_msgs/Twist`)
- Send `CMD_SET_DRIVE (0x10)` to STM32 (`v/w` in Q15)
- Send `CMD_SET_MODE (0x11)` to force closed-loop mode on startup
- Poll `CMD_GET_STATUS (0x02)` for heartbeat and diagnostics
- Publish `odom` and `odom -> base_link` TF

## Launch
```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch stm32_robot_bridge stm32_bridge.launch.py port:=/dev/ttyUSB1 baudrate:=115200
```

## Important note
Current firmware status payload returns command-space `v_q15/w_q15`, not explicit physical odometry (`v_est/w_est` in SI units).
So this node currently publishes odom by integrating commanded velocity (temporary solution).

For production SLAM/Nav, recommend firmware extension:
- add `v_est` / `w_est` / `yaw_est` (or wheel odometry) to `CMD_GET_STATUS` payload
- bridge then switches odom source from command integration to real feedback
