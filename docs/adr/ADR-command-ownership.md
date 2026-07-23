# ADR：最终命令所有权

状态：Superseded by Platform API 4 command authority contract。

决定由 `robot_control` 独占 `HostMotionCommand` 发布权。Nav2、遥控、测试与研究
节点只发布候选 `Twist`；`robot_control` 统一完成仲裁、有限值/范围校验、lease、
速度/加速度/jerk 限制、监督降级和 enable 意图。Bridge 与 Fake Base 只能消费
Host 候选，不得自行选择 ROS 子来源。下位机 `CommandManagement` 仍是整车控制来源
的最终仲裁者。

证据：`tests/deployment/test_architecture_v3.py` 扫描唯一发布者，控制策略和
rearm 测试覆盖抢占、超时、非法值与恢复。
