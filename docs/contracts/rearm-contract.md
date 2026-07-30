# Rearm contract

fault、ESTOP、timeout、断线、固件拒绝、严重 supervision 或 reboot 都会关闭 Host 运动。故障前或故障期间的 disable 不能作为恢复证据。

Bridge 等待新无故障 STATUS，发送新 sequence 的 disable，并只接受同一 wire session、received/applied sequence 精确匹配、带 `SESSION_VALID|RECEIVED|APPLIED`、无 rejected 位且 reject reason 为零的 ACK。确认前只以相同 sequence 重发 disable。

`robot_control` 独立清空候选、等待完整 source quiet window、等待 `WIRE_REARM_READY`，再要求新的操作者/任务回调并生成新 `command_epoch`。Navigation Guard 必须等旧公开 Goal 真正终止，并拒绝旧 UUID，才允许新 Goal 的 Twist 成为 Host 意图。
# Bridge command decision boundary

The Bridge ROS adapter never maps an arbitrary callback error directly to a vehicle
release. `BridgeCore` returns an explicit immutable decision. Idempotent duplicates,
retired sessions and out-of-order sequences are diagnosed without mutating the current
target. A conflicting duplicate or invalid value in the current active epoch clears the
target and requires rearm. STATUS/ACK timeout, firmware rejection, ESTOP, fault and
transport loss remain global release events.
