# ADR 0004：编码器与 IMU 融合

决策：轮式位姿由四轮 encoder count 增量和 SE(2) 中点积分产生，轮速用于 twist、异常和动态协方差；初始 EKF 仅融合 wheel vx 与 IMU gyro_z。IMU 字段质量独立判断，时钟使用 affine 映射。完整噪声参数必须由独立数据集标定后才能发布。
