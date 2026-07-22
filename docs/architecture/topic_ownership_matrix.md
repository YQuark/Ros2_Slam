# Topic 所有权

所有名字均为相对名称，表中 `/` 仅表示解析后的根 namespace。

| Topic | 类型 | 唯一发布者 |
| --- | --- | --- |
| `/chassis/command` | `robot_interfaces/ChassisCommand` | `robot_control` |
| `/chassis/control_state` | `robot_interfaces/ControlState` | `robot_control` |
| `/wheel/observation` | `robot_interfaces/WheelObservation` | Real Bridge 或 Fake Base |
| `/imu/observation` | `robot_interfaces/ImuObservation` | Real Bridge 或 Fake Base |
| `/chassis/state` | `robot_interfaces/ChassisState` | Real Bridge 或 Fake Base |
| `/chassis/firmware_info` | `robot_interfaces/FirmwareInfo` | Real Bridge 或 Fake Base |
| `/motion/safety_state` | `robot_interfaces/MotionSafetyState` | `robot_supervision` |
| `/wheel/odom` | `nav_msgs/Odometry` | `robot_state_estimation` |
| `/imu/data` | `sensor_msgs/Imu` | `robot_state_estimation` |
| `/odom` | `nav_msgs/Odometry` | `robot_localization` |

`/diagnostics` 允许多个发布者，但每个 `DiagnosticStatus.name` 必须由一个
节点拥有。
