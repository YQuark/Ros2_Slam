# Topic 所有权

所有名字均为相对名称，表中 `/` 仅表示解析后的根 namespace。

| Topic | 类型 | 唯一发布者 |
| --- | --- | --- |
| `/chassis/host_motion_command` | `robot_interfaces/HostMotionCommand` | `robot_control` |
| `/chassis/host_control_state` | `robot_interfaces/HostControlState` | `robot_control` |
| `/wheel/observation` | `robot_interfaces/WheelObservation` | Real Bridge 或 Fake Base |
| `/imu/observation` | `robot_interfaces/ImuObservation` | Real Bridge 或 Fake Base |
| `/chassis/link_state` | `robot_interfaces/ChassisLinkState` | Real Bridge 或 Fake Base |
| `/chassis/firmware_control_state` | `robot_interfaces/FirmwareControlState` | Real Bridge 或 Fake Base |
| `/chassis/firmware_info` | `robot_interfaces/FirmwareInfo` | Real Bridge 或 Fake Base |
| `/motion/supervision_state` | `robot_interfaces/MotionSupervisionState` | `robot_supervision` |
| `/platform/compatibility_state` | `robot_interfaces/PlatformCompatibilityState` | `robot_config` |
| `/navigation/guard_state` | `robot_interfaces/NavigationGuardState` | `robot_navigation_guard` |
| `/wheel/odom` | `nav_msgs/Odometry` | `robot_state_estimation` |
| `/imu/data` | `sensor_msgs/Imu` | `robot_state_estimation` |
| `/odom` | `nav_msgs/Odometry` | `robot_localization` |

`/diagnostics` 允许多个发布者，但每个 `DiagnosticStatus.name` 必须由一个
节点拥有。
