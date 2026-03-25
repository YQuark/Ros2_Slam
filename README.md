# ROS2_SLAM 上层工程（Astra Pro + YDLIDAR X2 + STM32）

本仓库是小车 ROS2 上层工程，目标是统一管理以下能力：

- 传感器：Astra Pro 深度相机、YDLIDAR X2
- 建图：`slam_toolbox`
- 导航：`nav2`
- 底盘控制：`stm32_robot_bridge`（对接 `YQuark/STM32_Robot`）

当前系统支持建图源一键切换：

- 摄像头建图：`mapping_source:=camera`
- 雷达建图：`mapping_source:=lidar`

## 1. 目录说明

```text
ros2_ws
├── src
│   ├── robot_bringup
│   ├── ydlidar_ros2_driver
│   ├── ros2_astra_camera
│   ├── stm32_robot_bridge
│   └── YDLidar-SDK
├── launch_scripts
├── SYSTEM_OVERVIEW.md
└── README.md
```

## 2. 环境依赖

- Ubuntu 20.04
- ROS2 Foxy
- 建议已安装：
  - `ros-foxy-navigation2`
  - `ros-foxy-nav2-bringup`
  - `ros-foxy-slam-toolbox`
  - `ros-foxy-depthimage-to-laserscan`
  - `python3-serial`

## 3. 编译

```bash
cd /home/robot/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source /home/robot/ros2_ws/install/setup.bash
```

## 4. 一键启动

进入脚本目录：

```bash
cd /home/robot/ros2_ws/launch_scripts
```

### 4.1 摄像头建图（推荐先用这个）

```bash
./start_mapping.sh camera
```

说明：
- 使用 Astra 深度图 -> `depthimage_to_laserscan` -> `/camera/scan` -> `slam_toolbox`
- RViz 可直接看地图、深度图和激光扇面

### 4.2 雷达建图

```bash
./start_mapping.sh lidar
```

说明：
- 启动前会自动执行雷达健康检查（检测 `Check Sum` / `/scan` 频率）
- 默认使用高精参数档位（`quality`）
- 使用 YDLIDAR `/scan` -> `slam_toolbox`

可选参数：

```bash
# 指定参数档位（auto|quality|precision|fast）
./start_mapping.sh lidar quality
./start_mapping.sh lidar precision

# 跳过健康检查（不建议）
./start_mapping.sh lidar quality --skip-lidar-check

# 不启动 RViz（显卡渲染异常时建议）
./start_mapping.sh lidar precision --no-rviz
```

### 4.3 保存地图

```bash
./save_map.sh my_map
```

### 4.4 导航（需已保存地图）

```bash
# 真实底盘
./start_navigation.sh /home/robot/ros2_maps/my_map.yaml

# 虚拟底盘联调（底盘未接入时）
./start_navigation.sh /home/robot/ros2_maps/my_map.yaml --fake-base
# 或
./start_nav_fake.sh /home/robot/ros2_maps/my_map.yaml
```

### 4.5 建图链路验收

```bash
./check_mapping_pipeline.sh
```

默认检查：`/scan` 频率、`/map` 发布、`map->odom` TF、RViz 订阅状态。

## 5. 常用排查

### 5.1 检查关键话题

```bash
source /opt/ros/foxy/setup.bash
source /home/robot/ros2_ws/install/setup.bash
ros2 topic list | grep -E "^/camera/depth/image_raw$|^/camera/scan$|^/scan$|^/map$"
```

### 5.2 摄像头模式没图时

- 确认启动的是 `./start_mapping.sh camera`
- 避免多开旧 launch（脚本已自动清理）
- 检查 USB 连接与供电稳定

### 5.3 雷达模式图小/不准时

- 先跑健康检查：`./check_lidar_health.sh`
- 如出现 `Check Sum`，优先排查 USB 线、供电、Hub
- 确认没有并发雷达进程占用 `/dev/ttyUSB0`

## 6. 关键入口

- 总入口：`src/robot_bringup/launch/system.launch.py`
- 详细说明：`SYSTEM_OVERVIEW.md`
