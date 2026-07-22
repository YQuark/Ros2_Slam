# stm32_robot_bridge

v0.4.0 只安装 upper v3 runtime：`bridge_node_v3.py` 以 `bridge_node` 名称执行。ROS adapter 消费编译后的 effective params，串口线程通过有界队列输出 domain frame，`BridgeCore` 负责 HELLO/STATUS/session ACK、安全状态与 rearm。

```bash
cd /home/robot/ros2_ws
./bin/robot base
./bin/robot check chassis
```

bridge 发布原始 `wheel/observation`、`imu/observation`、`chassis/state`、`chassis/firmware_info` 与 diagnostics。它不计算 odometry、不处理估计协方差，也不拥有任何 TF。协议规范见 `docs/interfaces/upper-protocol-v3.md`。

源码中的 v2 文件仅是 v0.4.x 黄金回归 fixture，不被 CMake 安装。beta4/v2 固件不能驱动本运行时。
