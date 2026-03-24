# stm32_robot_bridge

ROS2 bridge for the `YQuark/SLAM_ROBOT` serial protocol.

## Features
- Subscribe `cmd_vel` (`geometry_msgs/Twist`)
- Send `CMD_SET_DRIVE (0x10)` to STM32 (`v/w` in Q15)
- Send `CMD_SET_MODE (0x11)` to force closed-loop mode on startup
- Poll `CMD_GET_STATUS (0x02)` for heartbeat and diagnostics
- Publish `odom` and `odom -> base_link` TF
- Auto-detect a CP2102 USB serial adapter when `port:=auto`
- Retry serial open automatically when the adapter is not plugged yet

## Upper-layer interface contract (must keep stable)
- Input: `/cmd_vel` (`geometry_msgs/Twist`)
- Output: `/odom` (`nav_msgs/Odometry`) at >= 20 Hz (recommended)
- TF: `odom -> base_link` continuous and timestamp-aligned with odom
- Static TF expected by upper stack: `base_link -> laser_frame`

### Pre-integration acceptance checklist
- `/cmd_vel` timeout fallback to zero velocity works
- `/odom` header/frame IDs remain fixed (`odom` / `base_link`)
- `odom` timestamp monotonic, no large backward jumps
- Speed limits consistent with launch params (`max_linear`, `max_angular`)

## Launch
```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch stm32_robot_bridge stm32_bridge.launch.py port:=auto baudrate:=115200
```

## Important note
The current `SLAM_ROBOT` firmware binds `PC_Link_Init(&huart3)` to `USART3`, which is `115200 8N1`.
For a CP2102 adapter, wire it to the STM32 `USART3` pins:
- `CP2102 TX -> STM32 USART3_RX (PB11)`
- `CP2102 RX -> STM32 USART3_TX (PB10)`
- `GND -> GND`

The firmware `GET_STATUS` payload now includes normalized `v_est/w_est` feedback and four wheel encoder speeds.
This bridge uses that feedback for odom integration when fresh status frames are available, and falls back to command integration only if status is stale.
