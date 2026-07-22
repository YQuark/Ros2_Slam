# 运行拓扑

## 真实底盘

`robot_control + stm32_robot_bridge + robot_state_estimation +
robot_supervision + robot_localization`

## 虚拟底盘

只用 `robot_verification/fake_base` 替换 `stm32_robot_bridge`。其余节点、
Topic 类型、QoS、frame 和 TF 所有权不变。

## 无底盘

只允许传感器诊断和显式静态 TF 回退；导航模式禁止无底盘启动。
