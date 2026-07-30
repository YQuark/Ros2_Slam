# Topic 与 Action 所有权

名字按默认根 namespace 展示；代码和配置使用相对名称以支持 namespace。

| 名称 | 类型 | 唯一发布者/服务端 |
| --- | --- | --- |
| `/cmd_vel/teleop`、`/cmd_vel/test`、`/cmd_vel/nav`、显式 research topics | `geometry_msgs/Twist` | 各候选来源；不是底盘正式出口 |
| `/chassis/host_motion_command` | `HostMotionCommand` | `robot_control/cmd_vel_mux` |
| `/chassis/host_control_state` | `HostControlState` | `robot_control/cmd_vel_mux` |
| `/wheel/observation` | `WheelObservation` | Real Bridge 或 Fake Base，互斥 |
| `/imu/observation` | `ImuObservation` | Real Bridge 或 Fake Base，互斥 |
| `/chassis/link_state` | `ChassisLinkState` | Real Bridge 或 Fake Base，互斥 |
| `/chassis/firmware_control_state` | `FirmwareControlState` | Real Bridge 或 Fake Base，互斥 |
| `/chassis/firmware_info` | `FirmwareInfo` | Real Bridge 或 Fake Base，互斥 |
| `/platform/compatibility_state` | `PlatformCompatibilityState` | `platform_compatibility` |
| `/motion/supervision_state` | `MotionSupervisionState` | `motion_supervisor` |
| `/navigation/guard_state` | `NavigationGuardState` | `navigation_guard` |
| `/wheel/odom` | `nav_msgs/Odometry` | `wheel_odometry` |
| `/imu/data` | `sensor_msgs/Imu` | `imu_adapter` |
| `/odometry/filtered_internal` | `nav_msgs/Odometry` | `base_ekf`，仅 wheel_imu 模式 |
| `/odom` | `nav_msgs/Odometry` | `formal_odometry` |
| `/scan_raw` | `sensor_msgs/LaserScan` | YDLidar driver |
| `/scan` | `sensor_msgs/LaserScan` | `scan_normalizer` |
| `/chassis/operation` | `ChassisOperation` Action | `chassis_ops` |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` Action | `navigation_guard` |
| `/nav2/navigate_to_pose` | `nav2_msgs/NavigateToPose` Action | Nav2 backend |

`/diagnostics` 允许多个发布者，但每个 `DiagnosticStatus.name` 只能有一个语义所有者。
`/stm32_bridge/wire_*` 或 `/fake_base/wire_*` 是 provider 私有适配服务，只表示队列
接纳，不是公共“已应用”证据。
