# robot_bringup

Unified orchestration package for the robot upper stack.

## Main entry
```bash
ros2 launch robot_bringup system.launch.py mode:=mapping
```

## Modes
- `mode:=mapping`: lidar + base + slam + rviz
- `mode:=navigation`: lidar + base + localization + nav2 + rviz

## Common args
- `use_camera:=true|false`
- `use_base:=true|false`
- `base_mode:=real|fake|none` (recommended; legacy `use_base` kept for compatibility)
- `use_rviz:=true|false`
- `map_file:=/abs/path/map.yaml` (required for navigation)
- `base_port:=auto`
- `base_baudrate:=115200`
- `base_max_linear:=0.50`
- `base_max_angular:=1.50`

## Example
```bash
# mapping
ros2 launch robot_bringup system.launch.py mode:=mapping use_camera:=false

# navigation
ros2 launch robot_bringup system.launch.py \
  mode:=navigation \
  base_mode:=fake \
  map_file:=/home/robot/ros2_maps/my_map.yaml
```

## Nav2 Control Chain
- `nav2` publishes `/cmd_vel`
- `stm32_robot_bridge` subscribes `/cmd_vel`
- bridge sends drive commands to STM32 lower controller
- lower controller feedback closes the `/odom` + `odom->base_link` loop

## Note
If navigation dependencies are missing, launch will fail fast with install hints.
When `base_port:=auto`, the bridge prefers a CP2102/`cp210x` serial adapter and keeps retrying if it is not plugged yet.
