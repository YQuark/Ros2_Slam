# TF ownership

| Transform | Sole dynamic/static owner | Recovery owner |
| --- | --- | --- |
| `map -> odom` | SLAM or localization, mutually exclusive | bringup lifecycle |
| `odom -> base_link` | `formal_odometry` | compatibility-gated state-estimation bringup |
| `base_link -> base_footprint` | `robot_description` | robot_state_publisher |
| `base_link -> laser_frame` | `robot_description` | robot_state_publisher |
| `base_link -> imu_link` | `robot_description` | robot_state_publisher |

Bridge, Fake Base and wheel odometry never publish TF.
