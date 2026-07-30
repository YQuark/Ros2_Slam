# ADR 0003：TF 所有权

状态：Accepted；按 Platform API 5 修订。

决策：Bridge、Fake Base、wheel odometry 和内部 EKF 都不发布正式 TF；
`formal_odometry` 是 `odom → base_link` 唯一所有者。wheel-only 时它消费
`wheel/odom`，wheel+IMU 时消费 `odometry/filtered_internal`。原因是原始测量、
融合状态与正式兼容出口必须有清晰边界。

验证：launch/依赖边界测试与运行时 TF graph 报告。
