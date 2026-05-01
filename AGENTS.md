# AGENTS.md

> **仓库**: [YQuark/Ros2_Slam](https://github.com/YQuark/Ros2_Slam)
> **分支**: `ARM`（树莓派 4B 迁移分支，从 `master`(x86/Foxy) 分出，目标 ROS2 Humble）
> **下位机**: [YQuark/SlamRobot_Chassis_Control_V1.0](https://github.com/YQuark/SlamRobot_Chassis_Control_V1.0)
> **旧下位机原型**: [YQuark/SLAM_ROBOT](https://github.com/YQuark/SLAM_ROBOT)（四轮差速，已归档只读）

## 项目定位

本项目为视觉 SLAM 移动机器人树莓派上位机工程。目标是在原 x86 架构上位机仓库 `YQuark/Ros2_Slam` 的基础上，完成面向树莓派 4B 的迁移、裁剪、重构和工程化适配。

本工程不是简单复制旧 x86 上位机代码，而是围绕当前新硬件架构重新收口：

- 上位机：Raspberry Pi 4B
- 系统：Ubuntu 22.04 64-bit
- ROS2：默认 ROS2 Humble
- 激光雷达：YDLIDAR X2
- 下位机：STM32F407 两轮差速底盘控制板
- 下位机仓库：`https://github.com/YQuark/SlamRobot_Chassis_Control_V1.0`
- 通信方式：树莓派 UART 或 USB-UART 与 STM32 USART3 通信
- 当前底盘：两轮差速，不再沿用旧四轮、麦轮或四路编码器假设
- 当前目标：实现非自主建图、自主建图、定位导航，后期接入摄像头进行 AI 检测识别

本项目的核心任务是让树莓派稳定承担 ROS2 上位机职责：

1. 接入 YDLIDAR X2 激光雷达。
2. 接入 STM32 下位机串口协议。
3. 发布可靠 `/scan`、`/odom`、`/tf`、`/imu/data` 或相关状态话题。
4. 支持 slam_toolbox 建图。
5. 支持 map_server、AMCL 与 Nav2 定位导航。
6. 支持手动控制、非自主建图和自主建图。
7. 后续接入摄像头，实现 AI 目标检测、识别和行为联动。
8. 与下位机工程同步推进，协议、运动学、里程计、故障状态必须一致。

---

## 总体原则

### 1. 以当前硬件为真值来源

开发时必须以当前下位机工程和实际机器人硬件为准，不得沿用旧 x86 工程中的过时假设。

当前下位机关键事实：

- MCU：STM32F407
- 框架：HAL + FreeRTOS
- 底盘：两轮差速
- 电机：左右双电机，JGB37-520，12 V，178 RPM 档位
- 驱动芯片：TB67H450FNG,EL
- 编码器：AB 相霍尔编码器
- 编码器初始估算：`11 * 4 * 56 = 2464 counts/rev`
- 编码器最终参数必须以实际 TIM 编码器模式和实测为准
- ADC：电池电压采样、左电机电流采样、右电机电流采样
- IMU：MPU6050
- 树莓派通信口：USART3，默认 115200 8N1
- 调试串口：USART1
- ESP01S：USART2，作为预留通信路径，不是 ROS2 主链路

上位机不得假设：

- 四轮底盘
- 麦轮底盘
- 四路编码器
- 旧电机参数
- 旧驱动芯片
- 旧串口协议字段
- x86 上运行良好就等于树莓派上可直接运行

### 2. 以可运行链路优先

任何修改必须优先保证以下主链路可运行：

```text
YDLIDAR X2 -> /scan -> slam_toolbox / Nav2
STM32 USART3 -> stm32_robot_bridge -> /odom + /tf + /chassis/status
Nav2 / teleop -> /cmd_vel -> stm32_robot_bridge -> STM32 -> motor
```

不允许为了新增功能破坏主链路。

### 3. 以小步修改为原则

每次修改只应集中解决一个明确问题：

- 雷达接入
- 底盘串口
- TF
- odom
- slam_toolbox
- Nav2
- 自主建图
- 摄像头接入
- AI 检测
- 文档或脚本

不要一次性大规模重构多个核心包。

### 4. 以实测替代理论猜测

涉及以下内容时，必须实测或保留配置项：

- 轮径
- 轮距
- 编码器 counts/rev
- 左右轮方向符号
- 电机方向符号
- 雷达安装 yaw
- IMU 坐标系
- 电压采样比例
- 电流采样比例
- 最大速度
- 最大角速度
- Nav2 footprint
- costmap 安全距离

不确定参数只能标注“待实测”，不得写成确定值。

---

## 迁移目标

### 第一目标：树莓派替代 x86 上位机

树莓派上位机必须实现旧 x86 工程中的核心能力，但需要根据树莓派算力、arm64 依赖、串口设备名、功耗和散热重新适配。

必须保留的能力：

- YDLIDAR X2 雷达接入
- `/scan` 发布
- slam_toolbox 建图
- 地图保存
- map_server 地图加载
- AMCL 定位
- Nav2 导航
- `/cmd_vel` 到 STM32 的控制链路
- STM32 状态回传
- `/odom` 发布
- `base_link`、`odom`、`map`、`laser_frame` TF 链路
- RViz 可视化调试
- 无 RViz 运行模式
- 现场诊断脚本

必须重构或重新验证的能力：

- 串口自动识别
- STM32 协议解析
- 两轮差速里程计
- 电机方向符号
- 编码器方向符号
- 雷达安装方向
- 激光点云是否镜像
- 导航中机器人前进方向是否与 RViz 坐标一致
- 树莓派启动脚本
- systemd 自启动服务
- CPU、内存、温度占用
- Headless 模式运行稳定性

### 第二目标：形成可长期迭代的产品化上位机

上位机工程不应只服务一次调试，而应支持后续持续开发：

- 可重复部署
- 可快速诊断
- 可自动启动
- 可远程维护
- 可记录日志
- 可区分真实底盘与虚拟底盘
- 可区分手动建图、自主建图、定位导航和 AI 感知
- 可与下位机协议稳定同步

---

## 系统分层

推荐继续沿用旧仓库的分层思想，但在树莓派版本中进行清理。

```text
ros2_ws/
├── AGENTS.md
├── README.md
├── SYSTEM_OVERVIEW.md
├── docs/
│   ├── 00_project_scope.md
│   ├── 01_raspberrypi_setup.md
│   ├── 02_hardware_connection.md
│   ├── 03_lidar_mapping.md
│   ├── 04_chassis_bridge_protocol.md
│   ├── 05_navigation.md
│   ├── 06_autonomous_mapping.md
│   ├── 07_ai_camera_extension.md
│   └── 99_troubleshooting.md
├── launch_scripts/
│   ├── robot.sh
│   ├── check_system.sh
│   ├── check_lidar.sh
│   ├── check_chassis.sh
│   ├── check_tf.sh
│   ├── check_navigation.sh
│   └── install_service.sh
├── src/
│   ├── robot_bringup/
│   │   ├── launch/
│   │   ├── config/
│   │   ├── rviz/
│   │   └── behavior_trees/
│   ├── stm32_robot_bridge/
│   │   ├── src/
│   │   ├── include/
│   │   ├── launch/
│   │   └── config/
│   ├── robot_description/
│   │   ├── urdf/
│   │   └── meshes/
│   ├── robot_mapping/
│   │   ├── launch/
│   │   └── config/
│   ├── robot_navigation/
│   │   ├── launch/
│   │   ├── config/
│   │   └── maps/
│   ├── robot_exploration/
│   │   ├── launch/
│   │   └── src/
│   ├── robot_perception/
│   │   ├── launch/
│   │   ├── config/
│   │   └── src/
│   ├── ydlidar_ros2_driver/
│   └── third_party/
├── tools/
│   ├── serial_probe.py
│   ├── protocol_debug.py
│   ├── odom_calibration.py
│   ├── lidar_direction_check.py
│   ├── cpu_monitor.py
│   └── map_quality_check.py
└── scripts/
    ├── setup_udev_rules.sh
    ├── setup_ros_env.sh
    └── setup_rpi_performance.sh
```

要求：

- `launch_scripts/robot.sh` 是唯一正式运行入口。
- `ros2 launch` 可以用于开发调试，但文档和现场运行必须优先使用 `robot.sh`。
- `system.launch.py` 是 ROS2 编排核心，不应让多个 launch 文件重复写全量系统逻辑。
- `docs/` 必须随代码更新。
- 不要把诊断脚本、测试工具和正式 launch 混在一起。
- 不要把 x86 专属路径写死到树莓派版本中。
- 不要保留 `/home/robot/ros2_ws` 之外不可配置的绝对路径，除非明确是部署约定。

---

## ROS2 版本与系统约束

默认环境：

```text
Platform: Raspberry Pi 4B
Architecture: arm64
OS: Ubuntu 22.04 64-bit
ROS2: Humble
Workspace: /home/robot/ros2_ws
Shell: bash
DDS: Fast DDS 或 CycloneDDS，现场稳定性优先
```

开发要求：

- Ubuntu 22.04 默认使用 ROS2 Humble。
- 不应继续默认 source `/opt/ros/foxy/setup.bash`。
- 旧 x86 工程中所有 Foxy 路径、Foxy 参数、Foxy 专属行为必须检查。
- Python 包、C++ 包、launch 文件需要按 Humble 兼容性检查。
- 树莓派上必须优先保证稳定运行，不追求复杂视觉和大模型推理。
- RViz 可以远程运行在 PC 上；树莓派本体应支持 `--no-rviz` 无头运行。
- 默认不在树莓派上长时间开启高负载 GUI。
- 需要记录 CPU、内存、温度、雷达频率、odom 频率、TF 延迟。

---

## 硬件连接约束

### 1. YDLIDAR X2

YDLIDAR X2 是本阶段唯一正式激光雷达。

要求：

- 保留 YDLIDAR X2 驱动。
- 雷达通过 USB 接入树莓派。
- 雷达设备名必须通过 udev 规则固定，例如：

```text
/dev/ydlidar
```

禁止：

- 不要默认切换到 RPLIDAR。
- 不要默认切换到深度相机建图。
- 不要把旧 x86 的 `/dev/ttyUSB0` 直接写死。
- 不要假设雷达永远是 `/dev/ttyUSB0`。

必须支持配置：

```yaml
lidar_port: /dev/ydlidar
lidar_frame: laser_frame
lidar_baudrate: 115200
lidar_inverted: true 或 false
lidar_angle_min: 可配置
lidar_angle_max: 可配置
lidar_tf_x: 可配置
lidar_tf_y: 可配置
lidar_tf_z: 可配置
lidar_tf_yaw: 可配置
```

雷达方向必须通过实测确认：

- RViz 中机器人前方障碍物是否出现在 `base_link` 正前方。
- 原地旋转时激光点云与地图是否同向旋转。
- 导航时前进方向是否与地图坐标一致。
- 若建图正确但导航前后反向，优先检查 `base_link -> laser_frame` yaw、odom yaw、底盘方向符号，而不是盲目改 scan 数据。

### 2. STM32 下位机通信

STM32 是当前唯一正式底盘控制器。

通信链路：

```text
Raspberry Pi 4B UART / USB-UART  <->  STM32 USART3
```

推荐设备名：

```text
/dev/stm32_chassis
```

必须通过 udev 固定，不允许正式代码依赖临时 `/dev/ttyUSB0`、`/dev/ttyUSB1` 顺序。

串口默认参数：

```text
baudrate: 115200
data bits: 8
parity: none
stop bits: 1
flow control: none
```

上位机桥接节点必须支持：

订阅：

```text
/cmd_vel
```

发布：

```text
/odom
/tf
/chassis/status
/chassis/raw
/chassis/error
/battery_state
/motor/current
/imu/data 或 /chassis/imu
```

必须处理：

- 串口打开失败
- 下位机无响应
- 校验失败
- 帧头错位
- 状态帧超时
- 控制帧超时
- 下位机错误位
- 电池低压
- 左右电机过流
- 编码器异常
- IMU 数据未就绪

上位机不得做：

- 不要绕过 STM32 直接控制电机。
- 不要在 ROS2 节点中虚构电机状态。
- 不要在没有下位机回传的情况下发布看似真实的电流、电压、错误状态。
- 不要让 `/cmd_vel` 在串口断开时继续缓存输出。
- 不要让 Nav2 的速度命令在下位机超时停车后仍误判为可控。

### 3. 摄像头预留

摄像头属于后续 AI 检测识别扩展，不属于当前激光 SLAM 主链路。

要求：

- 摄像头节点默认关闭。
- 摄像头设备名必须可配置。
- 摄像头不得阻塞 `/scan`、`/odom`、`/tf` 和 `/cmd_vel`。
- AI 检测结果只发布感知话题，不得直接绕过决策层控制底盘。

---

## 上下位机同步开发原则

本上位机工程必须与下位机工程同步推进。

### 同步内容

以下内容一旦下位机修改，上位机必须同步：

- 串口帧格式
- 命令字
- 数据字段顺序
- 数据类型
- 缩放系数
- 校验算法
- 控制模式枚举
- 错误位定义
- 线速度和角速度单位
- 左右轮速度单位
- 编码器计数方向
- 电机方向符号
- 轮径
- 轮距
- 编码器 counts/rev
- IMU 坐标系
- 电压电流换算参数
- 低压、过流、堵转阈值

### 协议文件要求

上位机必须维护协议说明：

```text
docs/04_chassis_bridge_protocol.md
src/stm32_robot_bridge/config/chassis_protocol.yaml
```

下位机对应协议文档应保持一致（以代码为准）：

```text
SlamRobot_Chassis_Control_V1.0/App/protocol/upper_protocol.h  —— C 结构体定义（权威）
SlamRobot_Chassis_Control_V1.0/docs/protocols/2026-04-27-usart3-upper-protocol.md  —— 帧格式说明
```

任何协议变更必须同时说明：

```text
1. 改了什么字段
2. 为什么改
3. 是否兼容旧帧
4. 上位机需要怎么改
5. 下位机需要怎么改
6. 是否影响已有测试脚本
7. 是否需要重新标定 odom
```

### 推荐协议字段

上位机 bridge 实现必须以**当前下位机已实现的 v1 协议**为准，不得按目标协议虚构字段。

#### 当前协议（v1，下位机已实现）

控制帧 SET_VELOCITY (0x01)：

```text
frame_header(0xA5 0x5A) + length + cmd(0x01) + payload + checksum8
payload: linear_x(float) + angular_z(float) + enable(uint8) + mode(uint8)
```

急停帧 ESTOP (0x02)：

```text
frame_header(0xA5 0x5A) + length + cmd(0x02) + payload + checksum8
payload: enabled(uint8)
```

状态帧 STATUS (0x81)：

```text
frame_header(0xA5 0x5A) + length + cmd(0x81) + payload + checksum8
payload: left_speed(float) + right_speed(float)
       + left_encoder(int32) + right_encoder(int32)
       + battery_voltage(float) + left_current(float) + right_current(float)
       + imu_accel[3](int16) + imu_gyro[3](int16)
       + error_flags(uint32) + control_mode(uint8)
```

协议定义文件（下位机，以代码为准）：
- `App/protocol/upper_protocol.h` — C 结构体和常量
- `docs/protocols/2026-04-27-usart3-upper-protocol.md` — 帧格式说明

上位机 bridge 配置文件应与此严格对应：
- `src/stm32_robot_bridge/config/chassis_protocol.yaml`

#### 目标协议（v2，后续升级方向）

下位机增加 `version` / `seq` 字段后切换：
- 控制帧增加 `version + seq + reserved` 字段
- 状态帧增加 `version + seq + timestamp_ms` 字段
- IMU 数据改为融合值 `imu_gyro_z_radps + imu_yaw_rad` 替代原始 `imu_accel[3] + imu_gyro[3]`

当前阶段**不得**按 v2 协议实现 bridge。

---

## 运动学与里程计约束

当前机器人是两轮差速模型。

核心参数必须集中配置：

```yaml
wheel_radius_m: 待实测
wheel_base_m: 待实测
encoder_counts_per_rev: 待确认
left_encoder_sign: +1 或 -1
right_encoder_sign: +1 或 -1
left_motor_sign: +1 或 -1
right_motor_sign: +1 或 -1
odom_publish_rate_hz: 20
cmd_timeout_sec: 0.25
```

不得写死：

- 轮径
- 轮距
- 编码器 counts/rev
- 左右方向符号
- 电机最大速度
- PWM 与速度关系

上位机必须提供标定工具：

```text
tools/odom_calibration.py
```

至少支持：

- 前进 1 m 测试
- 原地旋转 360° 测试
- 左右轮编码器方向测试
- `/odom` 与实际位移对比
- `/tf` 中 `odom -> base_link` 方向检查
- Nav2 中机器人朝向检查

odom 验证标准：

```text
1. 手推或低速前进时，/odom x 增大。
2. 左转时，/odom yaw 按 ROS 坐标约定变化。
3. RViz 中 base_link 朝向与机器人真实前方一致。
4. /cmd_vel linear.x > 0 时，机器人真实前进。
5. /cmd_vel angular.z > 0 时，机器人按 ROS 约定逆时针旋转。
6. 建图、定位、导航三者使用同一套方向约定。
```

禁止：

- 不要通过同时反转多个符号来“碰巧修好”问题。
- 不要为了修建图镜像去破坏导航方向。
- 不要为了修导航方向去破坏 odom 坐标。
- 不要在没有记录验证结果时改动 TF yaw、laser inverted、odom sign。

---

## TF 坐标系要求

标准 TF 链路：

```text
map
└── odom
    └── base_link
        ├── base_footprint
        ├── laser_frame
        └── imu_link
```

要求：

- `map -> odom` 由 SLAM 或 AMCL 发布。
- `odom -> base_link` 由 STM32 bridge 或 EKF 发布。
- `base_link -> laser_frame` 由静态 TF 发布。
- `base_link -> imu_link` 由静态 TF 发布。
- 不允许多个节点重复发布同一条 TF。
- 不允许同时发布两个互相冲突的 `odom -> base_link`。
- EKF 未稳定前，导航默认优先使用 bridge odom。
- EKF 只能作为显式开关启用，不得在未验证时默认启用。

必须提供检查脚本：

```text
launch_scripts/check_tf.sh
```

检查内容：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /tf
ros2 topic echo /tf_static --once
```

---

## 建图目标

本工程需要同时支持非自主建图和自主建图。

### 1. 非自主建图

非自主建图指：

- 人通过键盘、手柄、网页或其他控制方式移动机器人。
- 上位机运行 YDLIDAR X2 + slam_toolbox。
- 机器人在人工控制下完成地图采集。
- 建图完成后保存地图。

推荐入口：

```bash
./robot.sh mapping lidar --manual --real-base
./robot.sh save-map classroom_v1
```

必须支持：

```text
/scan
/odom
/tf
/cmd_vel
/map
/slam_toolbox
```

要求：

- 手动控制速度必须限幅。
- 建图时默认速度低于导航速度。
- 手动控制超时必须停车。
- 保存地图必须同时保存 `.yaml` 与 `.pgm`。
- 地图文件命名要包含场景、日期或版本。

示例地图路径：

```text
src/robot_navigation/maps/classroom_v1.yaml
src/robot_navigation/maps/classroom_v1.pgm
```

### 2. 自主建图

自主建图指：

- 系统自动选择未知区域边界或目标点。
- 通过 Nav2 或探索节点驱动机器人移动。
- slam_toolbox 持续建图。
- 建图过程可人工中断。
- 建图完成后保存地图。

推荐入口：

```bash
./robot.sh mapping lidar --auto --real-base
```

自主建图必须分阶段实现。

#### 阶段 A：安全可控的半自动建图

- slam_toolbox 建图
- Nav2 可用
- 人工在 RViz 中点目标
- 机器人自动到达目标
- 人工选择下一个目标

#### 阶段 B：边界探索建图

- 引入 frontier exploration
- 自动选择未知边界
- 自动过滤过近障碍物目标
- 支持目标失败重试
- 支持卡死检测
- 支持低电压、过流、急停中断

#### 阶段 C：现场可用自主建图

- 可限制探索范围
- 可设置最大建图时间
- 可设置最大目标数量
- 可自动保存地图
- 可生成建图日志
- 可回放 `/scan`、`/odom`、`/tf`、`/cmd_vel`

自主建图安全约束：

```text
max_linear_speed <= 0.20 m/s
max_angular_speed <= 0.80 rad/s
obstacle_clearance >= 0.25 m
goal_clearance >= 0.30 m
cmd_timeout <= 0.25 s
```

具体数值后续按底盘实测调整，不得写死在多个文件中。

---

## 导航目标

导航目标是实现基于已保存地图的定位和目标点导航。

推荐入口：

```bash
./robot.sh navigation --real-base --map classroom_v1
```

导航链路：

```text
map_server
amcl
ydlidar_ros2_driver
stm32_robot_bridge
robot_state_publisher / static_tf
nav2_controller
nav2_planner
nav2_bt_navigator
rviz2
```

导航启动后必须检查：

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /cmd_vel
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

标准导航流程：

```text
1. 启动 navigation。
2. RViz 加载地图。
3. 使用 2D Pose Estimate 初始化位姿。
4. 确认激光点云与地图边界重合。
5. 下发 2D Goal Pose。
6. 观察 /cmd_vel、/odom、机器人真实运动是否一致。
7. 若机器人反向、横向或旋转方向异常，先停机，再检查 TF 与 odom sign。
```

禁止：

- 不要在初始位姿未对齐时直接发导航目标。
- 不要在 `/scan` 频率异常时调 Nav2 参数。
- 不要在 `/odom` 方向未验证时调 planner/controller。
- 不要用增大速度解决导航不稳。
- 不要在树莓派 CPU 过热降频时判断算法性能。

---

## 后续摄像头与 AI 检测识别

后续会引入摄像头进行 AI 检测识别，但当前阶段不得让摄像头影响激光 SLAM 主链路。

摄像头扩展原则：

- 第一阶段只做图像采集。
- 第二阶段做轻量目标检测。
- 第三阶段做检测结果与导航、行为联动。
- 不在树莓派上强行运行过重模型。
- 不让 AI 节点阻塞 SLAM、Nav2、底盘通信。

推荐包：

```text
src/robot_perception/
```

推荐话题：

```text
/camera/image_raw
/camera/camera_info
/perception/detections
/perception/target_status
/perception/debug_image
```

推荐功能：

- 摄像头启动
- 图像压缩传输
- 轻量检测模型推理
- 检测结果发布
- 识别目标类别、置信度、bbox
- 检测结果与导航状态解耦
- 可开关 AI 功能

禁止：

- 不要把 AI 检测写进 `stm32_robot_bridge`。
- 不要让 AI 节点直接发布 `/cmd_vel`，除非经过统一行为决策层。
- 不要让摄像头替代当前 YDLIDAR X2 主建图链路。
- 不要在未验证算力前默认开机启动 AI 模型。
- 不要把大模型、云端 API、复杂视觉 SLAM 引入当前主线。

---

## 统一运行入口

`launch_scripts/robot.sh` 是唯一正式入口。

必须支持以下命令结构：

```bash
./robot.sh check
./robot.sh check lidar
./robot.sh check chassis
./robot.sh check tf
./robot.sh check nav

./robot.sh mapping lidar --manual --real-base
./robot.sh mapping lidar --auto --real-base
./robot.sh mapping lidar --manual --fake-base
./robot.sh mapping lidar --auto --fake-base
./robot.sh mapping lidar --no-rviz

./robot.sh save-map MAP_NAME

./robot.sh navigation --real-base --map MAP_NAME
./robot.sh navigation --fake-base --map MAP_NAME
./robot.sh navigation --no-rviz --map MAP_NAME

./robot.sh teleop keyboard
./robot.sh teleop ps2

./robot.sh status
./robot.sh stop
```

参数要求：

```text
--real-base        使用真实 STM32 底盘
--fake-base        使用虚拟底盘，用于无硬件调试
--manual           非自主建图
--auto             自主建图
--no-rviz          不启动 RViz
--map MAP_NAME     指定地图
--lidar-port PATH  覆盖雷达串口
--base-port PATH   覆盖 STM32 串口
--lidar-yaw RAD    覆盖雷达 yaw
--ekf-base         显式启用 EKF
--bridge-base      使用 bridge odom
```

要求：

- 默认使用真实雷达。
- 默认使用真实底盘时必须检查串口是否存在。
- 没有雷达时不得伪装正常建图。
- 没有底盘时必须显式使用 `--fake-base`。
- `--ekf-base` 不得默认启用。
- 启动时必须打印当前关键配置：
  - ROS2 distro
  - base mode
  - lidar port
  - base port
  - map name
  - rviz mode
  - odom source
  - lidar TF
  - robot footprint
  - max speed limit

---

## 参数文件要求

所有核心参数必须集中管理。

推荐配置文件：

```text
src/robot_bringup/config/robot_common.yaml
src/robot_bringup/config/raspberrypi.yaml
src/robot_bringup/config/ydlidar_x2.yaml
src/stm32_robot_bridge/config/chassis_bridge.yaml
src/stm32_robot_bridge/config/chassis_protocol.yaml
src/robot_navigation/config/nav2_params_pi.yaml
src/robot_mapping/config/slam_toolbox_mapping.yaml
src/robot_mapping/config/slam_toolbox_localization.yaml
src/robot_exploration/config/exploration.yaml
src/robot_perception/config/camera.yaml
src/robot_perception/config/detector.yaml
```

不得散落硬编码：

- 串口路径
- 波特率
- 轮径
- 轮距
- 雷达 frame
- base frame
- odom frame
- map frame
- 最大线速度
- 最大角速度
- timeout
- 电池阈值
- 电流阈值
- AI 模型路径
- 摄像头设备路径

---

## 树莓派性能约束

树莓派是产品化低成本平台，不是 x86 工控机。

必须关注：

- CPU 占用
- 内存占用
- 温度
- 降频
- USB 设备稳定性
- 电源压降
- SD 卡写入
- ROS2 日志体积
- RViz 是否导致卡顿
- 多节点启动顺序

必须提供性能检查脚本：

```text
tools/cpu_monitor.py
launch_scripts/check_system.sh
```

检查内容：

```bash
vcgencmd measure_temp
vcgencmd get_throttled
free -h
df -h
top
ros2 node list
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom
```

要求：

- 长时间运行不应持续高温降频。
- 日志不得无限增长。
- rosbag 默认不自动开启。
- RViz 默认可关闭。
- 自主建图和 AI 检测不能默认同时高负载运行。
- 摄像头节点默认不进入主链路。

---

## 诊断优先级

现场问题必须按链路排查，不要直接改算法参数。

### 建图异常排查顺序

```text
1. /scan 是否存在，频率是否正常。
2. laser_frame 到 base_link 的 TF 是否正确。
3. /odom 是否存在，方向是否正确。
4. base_link 前方是否对应机器人真实前方。
5. slam_toolbox 是否收到 scan 与 odom。
6. 地图是否镜像。
7. 原地旋转时地图是否扭曲。
8. 再考虑 slam_toolbox 参数。
```

### 导航异常排查顺序

```text
1. 地图是否正确。
2. 初始位姿是否正确。
3. 激光点云是否贴合地图。
4. /odom 方向是否正确。
5. /cmd_vel 是否正常。
6. STM32 是否收到速度命令。
7. 机器人真实运动是否符合命令。
8. costmap 是否正常。
9. 再考虑 Nav2 参数。
```

### 串口异常排查顺序

```text
1. 设备节点是否存在。
2. udev 是否正确。
3. 波特率是否一致。
4. 是否有权限访问串口。
5. 下位机是否在发送状态帧。
6. 帧头是否正确。
7. 校验是否正确。
8. 是否有日志污染协议帧。
9. 超时停车是否触发。
```

### TF 异常排查顺序

```text
1. 是否存在 map、odom、base_link、laser_frame。
2. 是否有多个节点重复发布 odom -> base_link。
3. base_link -> laser_frame yaw 是否与安装方向一致。
4. /odom yaw 是否与机器人真实旋转方向一致。
5. RViz 中 RobotModel 朝向是否与真实前方一致。
6. 激光点云是否在机器人前方显示。
```

---

## 与下位机联调检查清单

每次下位机协议或运动控制修改后，上位机必须重新验证：

```text
[ ] /cmd_vel linear.x > 0 时机器人前进
[ ] /cmd_vel linear.x < 0 时机器人后退
[ ] /cmd_vel angular.z > 0 时机器人按 ROS 约定左转
[ ] /cmd_vel angular.z < 0 时机器人按 ROS 约定右转
[ ] 前进时左右编码器计数方向符合配置
[ ] 原地旋转时左右编码器计数方向相反
[ ] /odom x 与真实前进方向一致
[ ] /odom yaw 与真实旋转方向一致
[ ] /tf 中 odom -> base_link 连续无跳变
[ ] 通信中断后 STM32 自动停车
[ ] 上位机停止后 STM32 自动停车
[ ] 电池电压能正常上报
[ ] 左右电流能正常上报或至少原始 ADC 值正常
[ ] 错误位能正常上报
[ ] Nav2 输出 /cmd_vel 后底盘响应稳定
[ ] 急停或禁用控制后 /cmd_vel 不再驱动电机
[ ] 下位机重启后上位机能够检测并重新同步
```

联调记录必须包含：

```text
测试日期：
上位机 commit：
下位机 commit：
电池电压：
轮径参数：
轮距参数：
编码器 counts/rev：
雷达 yaw：
odom 来源：bridge / ekf
测试结果：通过 / 未通过
异常说明：
```

---

## 当前不要做的事情

除非明确要求，否则不要执行以下操作：

- 不要把树莓派版本继续按 x86 工控机性能设计。
- 不要默认使用 ROS2 Foxy。
- 不要默认保留旧四轮底盘逻辑。
- 不要引入麦轮模型。
- 不要替换 YDLIDAR X2。
- 不要把摄像头作为当前主建图传感器。
- 不要在 MCU 侧实现 SLAM。
- 不要在上位机虚构下位机状态。
- 不要在未确认协议前写死字段偏移。
- 不要把雷达串口和底盘串口顺序写死。
- 不要让多个节点同时发布 `odom -> base_link`。
- 不要让多个节点同时发布 `/cmd_vel`，除非有明确仲裁。
- 不要为了临时跑通而删除安全停车逻辑。
- 不要把 AI 检测加入默认启动链路。
- 不要一次性大改旧仓库所有包。
- 不要在没有验证命令的情况下提交“已完成导航”。

---

## 开发阶段规划

### 阶段 0：ARM 分支创建与基础验证

目标：在 `YQuark/Ros2_Slam` 创建 `ARM` 分支，验证基础编译。

任务：

1. 从 `main` 创建 `ARM` 分支：`git checkout -b ARM main`
2. 检查并替换 ROS2 Foxy 依赖为 Humble（arm64 可用性确认）
3. `colcon build --symlink-install` 验证编译通过
4. 确认 YDLIDAR X2 驱动在 arm64 下可编译

验收：

```bash
git branch  # 确认在 ARM 分支
colcon build --symlink-install
source install/setup.bash
ros2 node list
```

### 第一阶段：树莓派基础迁移

目标：让旧 x86 ROS2 工程能在树莓派上构建、启动、检查。

任务：

1. 清理 x86 绝对路径。
2. 改为 ROS2 Humble 环境。
3. 检查 arm64 依赖。
4. 检查 YDLIDAR X2 驱动是否能在树莓派编译运行。
5. 建立树莓派 setup 文档。
6. 建立 udev 规则。
7. 建立 `robot.sh check`。
8. 建立 `--no-rviz` 模式。
9. 建立性能监控脚本。

验收：

```bash
colcon build --symlink-install
source install/setup.bash
./launch_scripts/robot.sh check
ros2 node list
```

### 第二阶段：YDLIDAR X2 接入

目标：雷达稳定发布 `/scan`。

任务：

1. 固定 `/dev/ydlidar`。
2. 配置 YDLIDAR X2 参数。
3. 检查 `/scan` 频率。
4. 检查 laser frame。
5. 检查雷达安装方向。
6. 支持 `lidar_tf_yaw` 运行时覆盖。
7. 支持 `lidar_inverted` 配置。
8. 编写雷达方向检查文档。

验收：

```bash
./launch_scripts/robot.sh check lidar
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link laser_frame
```

### 第三阶段：STM32 Bridge 接入

目标：树莓派能通过 USART3 控制下位机并接收状态。

任务：

1. 固定 `/dev/stm32_chassis`。
2. 实现串口打开、读取、写入。
3. 实现控制帧发送。
4. 实现状态帧解析。
5. 发布 `/odom`。
6. 发布底盘状态。
7. 支持通信超时停车。
8. 支持错误位上报。
9. 支持协议调试工具。

验收：

```bash
./launch_scripts/robot.sh check chassis
ros2 topic echo /chassis/status
ros2 topic hz /odom
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" --once
```

### 第四阶段：非自主建图

目标：人工控制机器人完成激光建图。

任务：

1. 启动 YDLIDAR X2。
2. 启动 STM32 bridge。
3. 启动 slam_toolbox。
4. 启动 teleop。
5. 验证 `/scan + /odom + /tf -> /map`。
6. 支持地图保存。
7. 输出建图流程文档。

验收：

```bash
./launch_scripts/robot.sh mapping lidar --manual --real-base
./launch_scripts/robot.sh save-map classroom_v1
```

### 第五阶段：定位与导航

目标：基于保存地图完成 Nav2 导航。

任务：

1. map_server 加载地图。
2. AMCL 定位。
3. Nav2 bringup。
4. RViz 初始化位姿。
5. 发送目标点。
6. 验证 `/cmd_vel` 到 STM32。
7. 调整速度限幅和 footprint。
8. 编写导航排障文档。

验收：

```bash
./launch_scripts/robot.sh navigation --real-base --map classroom_v1
```

### 第六阶段：自主建图

目标：实现半自动到自主探索建图。

任务：

1. 先实现 RViz 多目标点半自动建图。
2. 再引入 frontier exploration。
3. 增加目标失败处理。
4. 增加卡死检测。
5. 增加自动保存地图。
6. 增加探索日志。
7. 增加安全边界配置。

验收：

```bash
./launch_scripts/robot.sh mapping lidar --auto --real-base
```

### 第七阶段：摄像头 AI 检测识别

目标：在不影响激光 SLAM 主链路的前提下接入摄像头。

任务：

1. 接入摄像头。
2. 发布图像话题。
3. 建立 AI 检测节点。
4. 输出检测结果。
5. 增加可视化调试图。
6. 与导航状态解耦。
7. 后续再做行为联动。

验收：

```bash
ros2 topic echo /perception/detections
ros2 topic hz /camera/image_raw
```

---

## 测试要求

每次修改必须至少说明测试方式。

推荐测试命令：

```bash
colcon build --symlink-install
source install/setup.bash

./launch_scripts/robot.sh check
./launch_scripts/robot.sh check lidar
./launch_scripts/robot.sh check chassis
./launch_scripts/robot.sh check tf

ros2 topic list
ros2 node list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
```

禁止提交未说明验证方式的核心链路修改。

核心链路包括：

- 雷达驱动
- STM32 bridge
- odom
- TF
- slam_toolbox
- Nav2
- robot.sh
- 协议解析
- 串口读写
- 自主建图
- AI 检测节点接入

---

## 文档要求

每次涉及功能变化，必须同步文档。

必须维护：

```text
README.md
SYSTEM_OVERVIEW.md
docs/00_project_scope.md
docs/01_raspberrypi_setup.md
docs/02_hardware_connection.md
docs/03_lidar_mapping.md
docs/04_chassis_bridge_protocol.md
docs/05_navigation.md
docs/06_autonomous_mapping.md
docs/07_ai_camera_extension.md
docs/99_troubleshooting.md
```

文档风格：

- 中文为主。
- 面向实际部署。
- 不写空泛介绍。
- 每个功能必须包含启动命令。
- 每个排障项必须包含检查命令。
- 不确定参数必须标注“待实测”。
- 不得伪造已经完成的功能。
- 不得写“理论支持”替代“已测试”。

---

## Git 提交规范

本项目 Git 提交必须使用：

```text
标准前缀首字母大写 + 中文详细提交标题 + 中文提交说明
```

提交标题格式：

```text
Prefix: 中文提交标题
```

允许使用的标准前缀：

```text
Feat: 新增功能
Fix: 修复问题
Refactor: 重构代码
Chore: 工程配置、依赖、脚本、非功能性调整
Docs: 文档修改
Style: 代码格式调整，不改变逻辑
Test: 测试脚本或测试用例
Perf: 性能优化
Build: 构建系统修改
Ci: 持续集成相关修改
Revert: 回退提交
```

提交说明必须使用中文，并说明：

```text
1. 本次修改了什么。
2. 为什么修改。
3. 是否影响树莓派部署。
4. 是否影响雷达、底盘串口、TF、odom、Nav2 或建图链路。
5. 是否需要同步修改下位机协议。
6. 是否需要重新标定轮径、轮距、方向符号或雷达 TF。
7. 已执行哪些验证命令。
8. 当前还有哪些未完成项。
```

提交示例：

```text
Feat: 新增树莓派上位机基础启动入口

- 新增 launch_scripts/robot.sh 的树莓派运行模式
- 增加 ROS2 Humble 环境检查
- 增加 real-base、fake-base、no-rviz 参数
- 启动时打印雷达串口、底盘串口、odom 来源和 RViz 模式
- 本次修改不影响下位机协议
- 已验证 colcon build 与 robot.sh check 能正常执行
```

```text
Fix: 修复 YDLIDAR X2 在树莓派上设备名不稳定的问题

- 新增 ydlidar udev 规则说明
- 将默认雷达端口改为 /dev/ydlidar
- 保留 --lidar-port 参数用于现场覆盖
- 本次修改影响雷达启动链路，不影响 STM32 协议
- 已验证 ros2 topic hz /scan 输出稳定
```

```text
Refactor: 重构 STM32 bridge 以适配两轮差速协议

- 移除旧四轮速度字段解析逻辑
- 新增左右轮编码器、左右轮速度、电池电压和电机电流状态解析
- 新增通信超时停车检测
- 本次修改需要与下位机 USART3 状态帧协议同步
- 已验证 /cmd_vel 下发和 /odom 发布链路
```

```text
Docs: 补充树莓派建图与导航联调流程

- 新增非自主建图启动流程
- 新增地图保存命令
- 新增导航初始化位姿说明
- 补充 /scan、/odom、TF 检查命令
- 本次修改不影响代码逻辑
```

禁止提交标题：

```text
update
fix
test
修改
修改代码
临时提交
树莓派
导航好了
1
```

禁止提交说明：

```text
无说明
只写“完成”
只写“修复bug”
只写“测试通过”但不说明测试命令
```

---

## 分支与提交策略

推荐分支：

```text
ARM                     树莓派迁移主分支（ROS2 Humble）
main                    旧 x86 Foxy 稳定版本（保留不动）
feature/lidar-x2         雷达接入
feature/stm32-bridge     下位机桥接
feature/nav2-pi          树莓派导航参数
feature/auto-mapping     自主建图
feature/perception-ai    摄像头与 AI 感知
```

合并要求：

- 主链路未验证时不要合并到 `ARM`。
- `main` 保留 x86 Foxy 版本，后续 ARM 稳定后可作为长期维护分支。
- 功能分支从 `ARM` 分出，合并前必须至少通过 `colcon build`。
- 涉及真实硬件的修改必须写明是否已在真实机器人上测试。
- 若只在 fake-base 下测试，必须明确写出。

---

## Codex / Agent 修改要求

Codex 或其他代码 Agent 修改本仓库时必须遵守：

1. 先阅读 `AGENTS.md`。
2. 先查看当前目录结构。
3. 先判断修改属于：
   - 树莓派迁移
   - 雷达接入
   - STM32 bridge
   - 建图
   - 导航
   - 自主建图
   - AI 感知扩展
   - 文档或脚本
4. 修改前必须确认是否影响：
   - `/scan`
   - `/odom`
   - `/tf`
   - `/cmd_vel`
   - STM32 协议
   - YDLIDAR 参数
   - Nav2 参数
   - slam_toolbox 参数
5. 不确定硬件参数时，只能使用配置项或 TODO，不能编造。
6. 涉及串口协议时必须同步更新协议文档。
7. 涉及方向、TF、odom 时必须给出验证命令。
8. 涉及导航参数时必须说明前置验证是否已完成。
9. 不要一次性大规模重构多个核心包。
10. 优先小步修改、小步验证、小步提交。
11. 不要删除旧功能，除非确认已被当前树莓派架构替代。
12. 不要把未完成能力写成已完成。
13. 不要为通过构建而屏蔽关键错误。
14. 不要用假数据代替下位机真实状态。
15. 每次修改后必须给出建议 Git 提交标题和提交说明。

Agent 回复格式必须包含：

```text
结论：
本次修改：
影响范围：
验证命令：
风险与未完成项：
建议提交标题：
建议提交说明：
```

若发现需求不完整，Agent 应优先给出可执行的最小方案，不要长时间停在澄清问题上。

---

## 最低验收标准

树莓派上位机 1.0 版本最低必须达到：

```text
[ ] 树莓派 Ubuntu 22.04 + ROS2 Humble 环境可复现
[ ] colcon build 成功
[ ] YDLIDAR X2 可稳定发布 /scan
[ ] STM32 bridge 可稳定连接 USART3
[ ] /cmd_vel 可控制底盘运动
[ ] /odom 发布方向正确
[ ] TF 链路完整
[ ] 非自主建图可完成
[ ] 地图可保存
[ ] Nav2 可加载地图
[ ] AMCL 可定位
[ ] RViz 可下发目标点导航
[ ] 通信超时可停车
[ ] check 脚本可定位常见问题
[ ] README 和 docs 与实际命令一致
```

2.0 版本目标：

```text
[ ] 自主建图可用
[ ] frontier exploration 可控
[ ] 自动保存地图
[ ] 建图日志可回放
[ ] 摄像头图像接入
[ ] AI 检测节点可独立运行
[ ] AI 结果可发布但不侵入底盘控制主链路
```

---

## 当前关键风险

### 1. x86 到树莓派迁移风险

风险：

- ROS2 版本差异
- arm64 依赖缺失
- 树莓派算力不足
- RViz 运行卡顿
- USB 串口顺序变化
- 电源不稳导致雷达或串口掉线

应对：

- 默认 ROS2 Humble
- 固定 udev 设备名
- 支持 `--no-rviz`
- 增加系统性能检查
- 文档明确树莓派部署流程

### 2. 上下位机协议不同步风险

风险：

- STM32 改了协议字段，上位机仍按旧字段解析
- 上位机发布错误 odom
- Nav2 根据错误 odom 控制机器人
- 导致导航失败甚至撞击

应对：

- 维护协议文档
- 维护 `chassis_protocol.yaml`
- 每次协议变更必须同步说明
- 增加协议调试工具
- 增加 frame version 和 checksum

### 3. 方向符号与 TF 风险

风险：

- 建图看似正确，导航前后反向
- 地图左右镜像
- 机器人旋转方向与 RViz 不一致
- laser frame yaw 与 base_link 不一致
- odom yaw 与真实运动相反

应对：

- 所有方向符号集中配置
- 提供 TF 检查脚本
- 提供 odom 标定脚本
- 每次改方向必须记录验证结果
- 不允许同时乱改多个方向参数

### 4. 自主建图安全风险

风险：

- 机器人探索目标过近
- 目标点贴墙
- costmap 不稳定
- 机器人卡死后继续输出速度
- 电池低压或过流仍继续运行

应对：

- 自主建图速度限幅
- 目标点 clearance 配置
- 卡死检测
- 超时停车
- 急停接口
- 低压、过流状态联动

### 5. 后续 AI 功能侵入主链路风险

风险：

- AI 节点占用树莓派算力
- 摄像头影响 USB 带宽
- 推理延迟影响导航
- AI 直接控制底盘导致危险

应对：

- AI 默认关闭
- AI 与 SLAM/Nav2 解耦
- AI 只发布检测结果
- 行为联动必须经过决策层
- 不允许 AI 节点直接绕过安全层控制底盘

---

## 交付物要求

每个阶段交付必须包含：

```text
[ ] 可运行代码
[ ] 启动命令
[ ] 配置文件
[ ] 验证命令
[ ] 排障说明
[ ] 当前限制
[ ] 建议 Git 提交信息
```

阶段完成不得只写“完成”，必须能通过命令复现。

---

## 一句话工程判断

本项目的主线不是“把 x86 工程搬到树莓派”，而是“以树莓派为产品化上位机、以下位机两轮差速协议为约束，重新收口激光建图、导航、自主探索和后续 AI 感知扩展”。
