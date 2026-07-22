# 上位机系统上下文

上位机以 Platform API 3 为稳定边界。候选 `Twist` 只能进入
`robot_control`；真实与虚拟底盘都消费 `ChassisCommand`，并提供相同的
`WheelObservation`、`ImuObservation`、`ChassisState` 和
`FirmwareInfo`。状态估计独占轮式里程计与正式 `odom` 链，运动监督只
输出降级/释放决策。

```text
Twist sources -> robot_control -> ChassisCommand -> Real/Fake base provider
                                                   |-> WheelObservation
                                                   |-> ImuObservation
                                                   `-> ChassisState

WheelObservation + ImuObservation -> robot_state_estimation
                                  -> wheel/odom + imu/data
                                  -> robot_localization -> odom + odom->base_link

ChassisCommand + observations -> robot_supervision -> MotionSafetyState
                                                    -> robot_control
```

Upper wire protocol v3 是 Bridge 与固件的私有传输接口，不得泄漏为状态
估计或控制包的内部类型依赖。
