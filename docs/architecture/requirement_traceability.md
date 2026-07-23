# 要求追踪

| ID | 要求 | 所有者 | 自动证据 | 硬件证据 |
| --- | --- | --- | --- | --- |
| HOST-CMD-001 | Host 候选命令唯一 ROS 出口 | `robot_control` | architecture graph test | 命令链 bag |
| HOST-CMD-002 | lease、限幅和 quiet window 使用单调时间 | `robot_control` | injected-clock tests | 停车时序记录 |
| WIRE-001 | Upper v3 codec/session/精确 ACK fail-closed | Bridge + firmware | codec/PTY tests | UART HIL |
| WIRE-002 | duplicate/out-of-order 不发布新观测 | Bridge | sample identity tests | 串口重放 |
| FW-CTRL-001 | 整车来源仲裁和物理运动许可归固件 | firmware | ownership contract | fault/源切换 HIL |
| OBS-001 | `enabled & valid & ~anomaly` 布局消费 | chassis model | M2+M3/4WD tests | 默认底盘 bag |
| ODOM-001 | 正式轮式里程计由状态估计拥有 | state estimation | layout/回绕/重启 tests | 直线与旋转标定 |
| SUP-001 | 监督只缩放或释放 Host 候选 | supervision | supervisor tests | 异常运动注入 |
| REARM-001 | 故障后新 disable + 精确 ACK + 新 Host 意图 | control + Bridge | rearm state-machine test | HIL fault injection |
| NAV-SAFE-001 | 撤权取消旧 Goal，恢复要求新 UUID | navigation guard | goal generation test | Nav2 fault HIL |
| CAL-001 | firmware commit/hardware/CRC/layout fail-closed | robot_config | compatibility tests | 参数 CRC 记录 |
| HIL-001 | 发布兼容必须绑定两个 commit 且 UART HIL PASS | release gate | manifest tests | HIL report |
| HOST-ARCH-002 | Bridge 不负责导航决策 | Bridge | dependency boundary test | 不适用 |
| HOST-ARCH-004 | `odom->base_link` 只有一个发布者 | formal_odometry | launch/TF graph test | runtime TF report |
| HOST-TEST-001 | Fake/Real 共用状态接口 | verification | graph contract test | Fake/Real manifest |

未通过硬件证据的要求不得因软件单测通过而标记为整机完成。
