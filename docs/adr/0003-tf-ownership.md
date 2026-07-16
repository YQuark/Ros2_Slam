# ADR 0003：TF 所有权

决策：bridge 永不发布 `odom → base_link`，EKF 是实车唯一所有者。原因是原始测量与融合状态必须有清晰边界。验证：launch 和运行时 ROS 图架构测试。
