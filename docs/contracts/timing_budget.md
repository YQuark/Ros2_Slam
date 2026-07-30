# 时序预算

| 项目 | 当前配置 | 时钟/语义 |
| --- | ---: | --- |
| Host 发布周期 | 50 ms | monotonic timer |
| DDS deadline | 100 ms | QoS 诊断 |
| DDS lifespan | 120 ms | 队列准入 |
| Bridge command/ACK timeout | 150 ms | monotonic fail-closed |
| Firmware Host watchdog | 200 ms | 固件独立安全边界 |
| STATUS timeout | 250 ms | monotonic fail-closed |
| mux 来源 lease | 250 ms | monotonic |
| command header 最大年龄 | 100 ms | ROS stamp 准入检查 |
| wire keepalive | 50 ms | 相同 target 可复用 sequence |

跨参数不变量由配置编译器执行：`publish < deadline <= lifespan < mux lease`、
`keepalive < bridge timeout < firmware timeout`、`ack <= bridge timeout`。

Host lease、supervision age、rearm quiet window、limiter `dt`、Bridge watchdog、
STATUS 和 ACK timeout 使用 monotonic time。ROS time 用于 header、Bag、观测映射和
命令入队年龄，`/clock` 跳变不得延长已接纳命令的安全 lease。每层停车时间必须在
UART HIL 分别测量，不能仅由配置值推断整车停车距离。
