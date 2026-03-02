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
- `use_rviz:=true|false`
- `map_file:=/abs/path/map.yaml` (required for navigation)
- `base_port:=/dev/ttyUSB1`
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
  map_file:=/home/robot/ros2_maps/my_map.yaml
```

## Note
If navigation dependencies are missing, launch will fail fast with install hints.
