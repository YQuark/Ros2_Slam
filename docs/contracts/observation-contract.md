# Observation contract

Bridge 只为新样本身份发布轮/IMU 观测。重复或乱序 STATUS 不发布 Observation、
不刷新新鲜度；消费者分别校验 `(wire_session, status_sequence)` 与
`(wire_session, sample_count, sensor_time)`。

线序固定为 LF/LR/RF/RR。Upper v3 当前没有独立 count-valid mask：累计 count 的
资格由 `motor_enabled_mask & ~encoder_anomaly_mask` 表示；轮速资格另为
`motor_enabled_mask & speed_valid_mask & ~encoder_anomaly_mask`。因此
`speed_valid_mask` 不得被解释为 count-valid。禁用轮不参与聚合，也不算故障。

积分每侧至少需要一个已接受轮。短时坏样本不提交 accepted baseline；两侧重新完整
后可从最后完整基线恢复累计位移。长间断、时间回退、session/reset generation 或
enabled topology 改变会显式重建 baseline，不尝试跨不可信间断补积分。单侧仅余一轮
时允许 DEGRADED 并增加 covariance；整侧不可用时不积分 pose。

MCU 样本时间用于积分并映射到 ROS 时间；Bridge 接收 Header 用于接收/Bag 事实，
估计器生成的 ROS stamp 用于 odom/TF。Bridge 写原始事实，状态估计可拒绝样本，固件
提供测量最终事实；会话恢复后由新观测重建状态。
