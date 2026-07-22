# ADR 0002：故障后重新使能

状态：Accepted。

背景：缓存命令可能在故障恢复后复动。

决策：断连、STATUS/ACK/command timeout、ESTOP、fault 或 supervisor critical
后锁存 `rearm_required`。恢复序列固定为：

```text
fault -> publish/hold disable -> clear candidate cache
      -> WAIT_SOURCE_QUIET -> WAIT_FRESH_SOURCE
      -> fresh command creates a new command session and enable edge
```

状态恢复、旧 Nav2 Twist、旧 ROS command sequence 和旧 wire session 都不能
直接重新驱动车辆。Bridge 每次建立新串口 generation 都先生成新 wire session，
连续发送 wire disable，并等待上层新的 disable。远程 ESTOP 只能置位不能释放。

验证：control rearm 单测、BridgeCore generation/timeout 单测、Upper-v3 PTY
reconnect 测试与 HIL fault injection。
