# 运行拓扑

`system.launch.py` 当前只接受 `mode:=mapping|navigation`、
`base_mode:=real|fake|none` 和 `base_fusion_mode:=wheel|wheel_imu`；`none`、`ekf`
仅作为旧 fusion alias。Base provider 存在时必须提供编译后的 effective params。

## Real Base

```text
robot_description + robot_sensing
+ robot_config/platform_compatibility
+ robot_chassis_ops
+ robot_control
+ stm32_robot_bridge
+ robot_state_estimation
+ robot_supervision
+ SLAM 或 AMCL/Nav2
```

## Fake Base

仅用 `robot_verification/fake_base` 替换 `stm32_robot_bridge`。消息类型、状态门、
估计、监督和 TF 所有权保持不变。Fake 场景必须显式记录 `simulated: true`，不能作为
物理 HIL 或车辆证据。

## No Base

只允许 mapping。可显式启用 `odom -> base_link` 静态回退来检查雷达/SLAM；
navigation 对 `base_mode:=none` 直接拒绝启动。静态回退不是里程计证据。

## Fusion

- `wheel`：`wheel/odom -> formal_odometry -> odom + TF`，当前默认。
- `wheel_imu`：`wheel/odom + imu/data -> base_ekf -> filtered_internal -> formal_odometry`，实验模式。

两条路径始终只有 `formal_odometry` 发布正式 `/odom` 和动态
`odom -> base_link`。
