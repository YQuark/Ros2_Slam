# stm32_robot_bridge

`stm32_robot_bridge` 是当前工程的底盘串口桥接包，负责把上位机运动命令转换为下位机串口协议，并把下位机状态反馈转换成 ROS2 话题与 TF。

## 包职责

- 订阅 `/cmd_vel`
- 发送 `SET_DRIVE` 和 `SET_MODE`
- 轮询 `GET_STATUS`
- 发布 `/odom`
- 按需发布 `/imu/data`
- 在未启用 EKF 时发布 `odom -> base_link`

## 在工程中的位置

- 用户入口不直接调用本包，优先使用 `/home/robot/ros2_ws/launch_scripts/robot.sh`
- 包级 launch 入口：`launch/stm32_bridge.launch.py`
- 工程编排入口通过 `robot_bringup/launch/base.launch.py` 引入本包

## 当前协议假设

- 串口默认参数：`115200 8N1`
- 默认工作模式要求下位机进入 `MODE_CLOSED_LOOP`
- 状态包优先使用二进制 `GET_STATUS`
- `/odom.twist` 默认使用状态包中的 `v_est / w_est`
- 桥接默认速度归一化与当前 STM32 固件物理归一化保持一致：`max_linear=1.20m/s`、`max_angular=19.27rad/s`
- 当前默认 `odom_angular_scale=1.0`，不再对最新固件的 `w_est` 做二次补偿
- 四轮编码器 cps 路径仍可通过 `odom_feedback_source:=wheel_cps` 用于 A/B 对照和故障诊断
- `use_status_yaw:=true` 时只在下位机 `imu_valid=1` 的状态帧中使用 `yaw_est`
- IMU 仅作为航向辅助和 EKF 输入，不直接承担平移积分
- `raw_accel_mg[3]` 只做 `/imu/data` 透传和健康检查，不做平移积分
- 当前固件导出的 `yaw_est / gz / raw_accel` 已经映射到车体系，桥接不要再额外做一次安装旋转

## 自动探测与启动行为

- `port:=auto` 时会优先扫描 CP210x 候选串口
- 候选串口会主动发送 `GET_STATUS` 进行协议级探测
- 串口连通后会先下发闭环模式和零速同步，再进入正常轮询

## 上层接口约定

- 输入：`/cmd_vel` (`geometry_msgs/Twist`)
- 输出：`/odom` (`nav_msgs/Odometry`)
- 可选输出：`/imu/data` (`sensor_msgs/Imu`)
- TF：`odom -> base_link`

## 常用调试方式

单独启动底盘桥接：

```bash
source /opt/ros/foxy/setup.bash
source /home/robot/ros2_ws/install/setup.bash
ros2 launch stm32_robot_bridge stm32_bridge.launch.py port:=auto baudrate:=115200
ros2 launch stm32_robot_bridge stm32_bridge.launch.py port:=auto baudrate:=115200 use_status_yaw:=true
ros2 launch stm32_robot_bridge stm32_bridge.launch.py port:=/dev/ttyUSB1 status_log_interval_sec:=1.0 cmd_log_interval_sec:=1.0
```

通过统一入口启动：

```bash
cd /home/robot/ros2_ws/launch_scripts
./robot.sh base
```

## 相关文档

- [/home/robot/ros2_ws/docs/06-底盘与串口桥接.md](/home/robot/ros2_ws/docs/06-底盘与串口桥接.md)
- [/home/robot/ros2_ws/src/robot_bringup/README.md](/home/robot/ros2_ws/src/robot_bringup/README.md)
