# ADR：跨层 rearm 语义

状态：Superseded by `docs/contracts/rearm-contract.md`。

通信层通过 `ChassisLinkState.rearm_required/rearm_reason_flags` 报告锁存原因；运动层
通过 `MotionSupervisionState.release_host_candidate` 报告 Host 释放请求；控制层通过
`HostControlState` 公开 gate、命令年龄、拒绝原因和 enable 意图。任何下层恢复都
不能自行清除控制层 gate。

恢复过程只有一个：控制层先发出新的 disable 并清空候选，候选来源再静默一个
lease 窗口，随后收到新的候选输入并创建新 command epoch。这样既阻止旧 Nav2 Goal/Twist
复用，也让 Real/Fake provider 使用同一恢复语义。

证据：`test_transport_rearm_clears_old_goal_and_requires_quiet_then_fresh_input`、
Bridge generation 测试和 HIL fault matrix。
