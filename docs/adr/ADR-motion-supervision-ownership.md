# ADR：运动监督所有权

状态：Accepted；包边界采用独立 `robot_supervision`，可在实车数据后复审。

四轮一致性、目标跟踪、非预期运动和 wheel/gyro yaw 差异不是传输事实，不能
由 Bridge 拥有。`robot_supervision` 订阅最终命令、`WheelObservation` 与质量
处理后的 IMU，发布 `MotionSafetyState`；它只给出分级、缩放和 release 建议。
最终命令与 rearm 状态仍由 `robot_control` 决定，Nav2 Goal 的取消属于后续导航
适配层。

证据：监督纯算法测试、Fake fault scenario 和 Topic 所有权测试。
