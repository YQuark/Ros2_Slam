# ADR：传输安全边界

状态：Accepted。

`stm32_robot_bridge` 仅拥有串口有界 I/O、framing、Upper v3 codec、HELLO、
capability、wire session/sequence/ACK、命令新鲜度、STATUS 超时和通信诊断。
任何缺少 HELLO、能力不完整、session/ACK 不一致、状态超时或写失败都必须
fail-closed 并请求 rearm。重复与乱序 STATUS 可作为原始观测发布以保留事实，
但不得推进 Bridge 状态或伪造新的采样序号。

Bridge 不计算 odometry、协方差、长期运动质量、TF 或 Nav2 决策。

证据：BridgeCore、framing/codec、PTY reconnect 和架构禁止依赖测试。
