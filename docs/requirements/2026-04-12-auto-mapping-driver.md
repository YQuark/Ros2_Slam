# Nav2 Frontier Auto Mapping Requirement

## Goal

Add an optional host-side frontier exploration mode so lidar SLAM can build maps
through Nav2 planned motion instead of PS2, ESP, or a direct `/cmd_vel` script.

## Constraints

- Auto mapping must be opt-in only.
- Existing manual mapping behavior must remain unchanged unless explicitly
  enabled.
- `--auto-drive` must launch Nav2 navigation servers and a frontier explorer.
- The frontier explorer must not publish `/cmd_vel`; only Nav2 may command the
  base during automatic mapping.
- `robot_bringup/launch/nav2.launch.py` must explicitly pass the behavior tree
  XML path to Nav2 Foxy `navigation_launch.py`; otherwise Foxy rewrites
  `default_bt_xml_filename` and can run a tree that does not match this
  package's `plugin_lib_names`.
- The mapping behavior tree must use only BT nodes whose plugin libraries are
  listed in the mapping Nav2 params.
- Nav2's map static layer must keep transient-local map subscription enabled in
  mapping so it consumes `slam_toolbox` `/map` reliably.
- Lidar mapping must feed SLAM/Nav2 from a fixed-size `/scan` stream; the raw
  YDLIDAR scan may vary by a few bins and should remain available as
  `/scan_raw`.
- `slam_toolbox` remains the map/localization source in mapping mode. AMCL and
  map_server must not be started for auto mapping.
- The explorer must cluster free map cells adjacent to unknown cells from
  `/map`, filter small/noisy frontier fragments, choose a safe known-free
  approach point behind a real unknown boundary, and send it to Nav2
  `NavigateToPose`.
- Nav2 must use conservative mapping-specific velocity and costmap parameters.
- Failed, rejected, or timed-out frontier goals must be blacklisted temporarily.
- While `/cmd_vel` is active, the STM32 bridge must keep requesting
  `MODE_CLOSED_LOOP` if the controller reports a non-closed-loop mode.
- Auto-drive runs must expose enough logs to distinguish no map, no TF, no
  frontier, Nav2 unavailable, rejected goals, failed goals, non-PC source, and
  non-closed-loop mode.
- Auto-drive with a real base must fail early if the STM32 base port is missing
  or resolves to the same device as the lidar port.
- The bridge must log both `/cmd_vel` receipt and serial `SET_DRIVE` writes so
  no-motion failures can be separated into ROS topic, bridge, serial, or MCU
  control-source problems.

## Acceptance Criteria

- `./robot.sh mapping lidar --real-base --ekf-base` remains manual.
- `./robot.sh mapping lidar --real-base --ekf-base --auto-drive` starts the
  Nav2 frontier explorer.
- Auto-drive prints `explorer_state` at a low default cadence so the useful
  state is visible without flooding the launch log.
- `bt_navigator` must not abort goals with
  `One of the children of a DecoratorNode or ControlNode is nullptr`.
- The explorer sends `NavigateToPose` goals to `/navigate_to_pose`.
- Nav2 publishes `/cmd_vel`; the explorer does not.
- The explorer reports frontier counts, rejection counts, active goal, and
  blacklist size in logs.
- A failed or timed-out target is blacklisted and the explorer chooses another
  frontier.
- Auto-drive enables STM32 `status_summary` logs at a low default cadence unless
  the operator overrides `BASE_STATUS_LOG_INTERVAL_SEC`.
- Auto-drive enables STM32 bridge `cmd_vel_rx` and `drive_tx` logs at a low
  default cadence unless the operator overrides `BASE_CMD_LOG_INTERVAL_SEC`.
- If the STM32 rejects a command, the bridge log includes the decoded NACK
  reason such as `CTRL_MODE_REJECT`.

## Remote Verification

当前目录是挂载环境，不能在本地执行，请在远程机器人通过 SSH 执行。

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup stm32_robot_bridge
source install/setup.bash
./launch_scripts/robot.sh mapping lidar --real-base --ekf-base --auto-drive
```

Optional observation commands on the robot:

```bash
ros2 topic echo /cmd_vel
ros2 action info /navigate_to_pose
ros2 topic echo /map --once
```
