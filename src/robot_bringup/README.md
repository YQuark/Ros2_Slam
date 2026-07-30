# robot_bringup

`robot_bringup` is the platform composition package. It combines description,
sensing, control, base bridge, state estimation, SLAM and Nav2 profiles; it does
not own device drivers, serial protocol logic, TF geometry or command
arbitration internals.

## Responsibilities

- Compose `mode:=mapping` and `mode:=navigation`.
- Keep mapping and navigation YAML/BT profiles in one place.
- Keep RViz as an explicit development option only.
- Preserve a single operator-facing launch surface for `robot.sh`.

## Not Owned Here

- Lidar driver and `/scan_raw -> /scan`: `robot_sensing`.
- Static platform frames: `robot_description`.
- `/cmd_vel/* -> /chassis/host_motion_command`: `robot_control`（仅 Host 候选）。
- `/wheel/odom`, optional internal EKF and formal `/odom`/TF gate: `robot_state_estimation`.
- STM32 serial protocol and raw base status: `stm32_robot_bridge`.

## Launch Entry Points

```bash
ros2 launch robot_bringup system.launch.py mode:=mapping base_mode:=none use_rviz:=false
ros2 launch robot_bringup system.launch.py mode:=mapping base_mode:=real effective_params_file:=/path/to/ros-params.yaml
ros2 launch robot_bringup system.launch.py mode:=navigation base_mode:=real effective_params_file:=/path/to/ros-params.yaml map_file:=/home/robot/robot_data/maps/latest.yaml
```

`use_rviz` defaults to `false`; pass `use_rviz:=true` only on a development
machine or when the Raspberry Pi has a display session.

## Stable Platform Topics

- `/scan_raw`: lidar driver output, internal to the platform.
- `/scan`: normalized 2D scan, public platform interface.
- `/wheel/observation`: raw wheel facts from either Real Base or Fake Base.
- `/imu/observation`: raw IMU facts from either Real Base or Fake Base.
- `/wheel/odom`: wheel odometry produced only by `robot_state_estimation`.
- `/imu/data`: quality-filtered IMU produced only by `robot_state_estimation`.
- `/odom`: public odometry published only by `formal_odometry`.
- `/odometry/filtered_internal`: optional wheel+IMU EKF output; never the public TF owner.
- `/cmd_vel/teleop`, `/cmd_vel/nav`, `/cmd_vel/test`: command candidates.
- `/cmd_vel/research/<name>`: optional whitelisted research command candidates.
- `/chassis/host_motion_command`: only `robot_control` should publish this.
- `/chassis/host_control_state`: Host selection, age, rejection and rearm state.
- `/motion/supervision_state`: Host motion-quality advice from `robot_supervision`.

Real/Fake providers require compiled effective params and are normally started through
`bin/robot`. Navigation rejects `base_mode:=none`. `base_fusion_mode:=wheel` is the
default; `wheel_imu` is experimental until its independent report passes.
