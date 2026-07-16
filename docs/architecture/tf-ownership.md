# TF 与测量所有权

- `wheel/odom`：编码器增量里程计，`odom`/`base_link`，不发布 TF。
- `imu/data`：STM32 IMU，orientation 未验证时 covariance[0] 为 -1。
- `odom`：EKF 融合输出，唯一发布 `odom → base_link`。
- `map → odom`：SLAM 或 localization 节点所有。

生产 launch 不允许第二个 `odom → base_link` 发布者。
