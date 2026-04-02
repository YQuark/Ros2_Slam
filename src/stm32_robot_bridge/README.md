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
- 状态包优先使用下位机返回的 `v_est / w_est`
- IMU 仅作为航向辅助和 EKF 输入，不直接承担平移积分

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
```

通过统一入口启动：

```bash
cd /home/robot/ros2_ws/launch_scripts
./robot.sh base
```

## 相关文档

- [/home/robot/ros2_ws/docs/06-底盘与串口桥接.md](/home/robot/ros2_ws/docs/06-底盘与串口桥接.md)
- [/home/robot/ros2_ws/src/robot_bringup/README.md](/home/robot/ros2_ws/src/robot_bringup/README.md)
