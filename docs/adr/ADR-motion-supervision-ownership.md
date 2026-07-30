# ADR：运动监督所有权

状态：Accepted；按 Platform API 5 术语修订。

四轮一致性、目标跟踪、非预期运动和 wheel/gyro yaw 差异不是传输事实，不能
由 Bridge 拥有。`robot_supervision` 订阅 Host 候选、`WheelObservation` 与质量
处理后的 IMU，发布 `MotionSupervisionState`；它只给出分级、缩放和释放 Host
候选的建议。下位机仍拥有物理安全；Nav2 Goal 取消由 `robot_navigation_guard` 负责。

证据：监督纯算法测试、Fake fault scenario 和 Topic 所有权测试。
# rc2 terminology and freshness

`robot_supervision` publishes deterministic `motion_consistency_risk` components and
Host degradation advice. The API-5 field `score` is retained for compatibility but is
not a slip probability. Stale gyro is excluded from yaw consistency; stale command is
excluded from target tracking and unexpected-motion classification; incomplete wheel
quality only affects components that remain observable. Threshold changes require a
labelled ROC/FPR/FNR report and independent validation.
