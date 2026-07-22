# ADR：最终命令所有权

状态：Accepted。

决定由 `robot_control` 独占 `ChassisCommand` 发布权。Nav2、遥控、测试与研究
节点只发布候选 `Twist`；`robot_control` 统一完成仲裁、有限值/范围校验、lease、
速度/加速度/jerk 限制、监督降级和 enable 意图。Bridge 与 Fake Base 只能消费
最终命令，不得自行选择命令来源。

证据：`tests/deployment/test_architecture_v3.py` 扫描唯一发布者，控制策略和
rearm 测试覆盖抢占、超时、非法值与恢复。
