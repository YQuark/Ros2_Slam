# ADR 0004：编码器与 IMU 融合

决策：轮式位姿由四轮 encoder count 增量和稳定的 SE(2)
指数映射积分产生（小角度使用级数展开），轮速用于 twist、异常和动态
协方差。生产默认为 wheel-only；显式 `wheel_imu` 实验融合 wheel
`vx/wz` 与 IMU `gyro_z`。IMU 字段质量独立判断，时钟使用 affine
映射。完整噪声参数必须由独立数据集标定后才能发布。
# rc2 evidence policy

The production default is direct formal wheel odometry and starts no EKF process.
`wheel_imu` is explicit and uses wheel `vx/wz` plus IMU `gyro_z`; if IMU times out,
wheel yaw rate remains available. A/B/C/D configurations are evaluated on identical
independent bags before changing the default. IMU validity is field-level: an invalid
gyro or accelerometer sets that field covariance to `-1` and does not discard other
valid fields. Without a magnetometer, IMU yaw is not treated as long-term absolute
heading.
