# 要求追踪

| ID | 要求 | 所有者 | 自动证据 | 硬件证据 |
| --- | --- | --- | --- | --- |
| HOST-ARCH-001 | 最终底盘命令唯一出口 | `robot_control` | architecture graph test | 命令链 bag |
| HOST-ARCH-002 | Bridge 不负责导航决策 | Bridge | dependency boundary test | 不适用 |
| HOST-ARCH-003 | 正式轮式里程计由状态估计拥有 | state estimation | import/publisher contract | wheel dataset |
| HOST-ARCH-004 | `odom->base_link` 只有一个发布者 | robot_localization | launch/TF graph test | runtime TF report |
| HOST-ARCH-005 | 配置只有一个事实源 | `robot_config` | compiler/schema test | manifest hash |
| HOST-SAFE-001 | 恢复后不得自动恢复旧命令 | control + Bridge | rearm state-machine test | HIL fault injection |
| HOST-TEST-001 | Fake/Real 共用状态接口 | verification | graph contract test | Fake/Real manifest |
| HOST-PROTO-001 | Upper v3 session/ACK fail-closed | Bridge + firmware | codec/PTY tests | UART HIL |

未通过硬件证据的要求不得因软件单测通过而标记为整机完成。
