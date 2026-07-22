# TF 所有权

| Transform | 发布者 |
| --- | --- |
| `map -> odom` | SLAM 或定位系统，二选一 |
| `odom -> base_link` | `robot_localization` |
| `base_link -> base_footprint` | `robot_description` |
| `base_link -> laser_frame` | `robot_description` |
| `base_link -> imu_link` | `robot_description` |

Bridge、Fake Base、wheel odometry 节点都不得发布 TF。

`wheel/observation` 和 `imu/observation` 是 Base provider 的原始事实，不带
TF 所有权。`wheel/odom` 由状态估计生成但不发布 TF；`imu/data` 也由状态估计
完成单位、时间与质量处理。生产 launch 不允许第二个 `odom -> base_link`
发布者。
