# Observation contract

Bridge 只为新样本身份发布轮/IMU 观测。重复或乱序 STATUS 只影响诊断，不刷新新鲜度；消费者分别校验 `(wire_session, status_sequence)` 与 `(wire_session, sample_count, sensor_time)`。

线序固定为 LF/LR/RF/RR。`enabled = motor_enabled_mask`，`valid = enabled & speed_valid_mask`；禁用轮不参与聚合，也不算故障。编码器积分还排除 anomaly，每侧至少需要一个有效轮。enabled/valid mask、wire session 或 reset generation 变化都会重建积分基线。

MCU 样本时间用于积分并映射到 ROS 时间；ROS Header 只服务消息、Bag 和 TF 时间。Bridge 写原始事实，状态估计可拒绝样本，固件提供测量最终事实；会话恢复后由新观测重建状态。
