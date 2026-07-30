# ADR：传输安全边界

状态：Accepted。

`stm32_robot_bridge` 仅拥有串口有界 I/O、framing、Upper v3 codec、HELLO、
capability、wire session/sequence/ACK、命令新鲜度、STATUS 超时和通信诊断。
任何缺少 HELLO、能力不完整、session/ACK 不一致、状态超时或写失败都必须
fail-closed 并请求 rearm。重复 STATUS 幂等忽略，乱序 STATUS 拒绝并诊断；两者都
不得发布新的 Observation、推进 Bridge 状态或刷新新鲜度。

Bridge 不计算 odometry、协方差、长期运动质量、TF 或 Nav2 决策。

证据：BridgeCore、framing/codec、PTY reconnect 和架构禁止依赖测试。
