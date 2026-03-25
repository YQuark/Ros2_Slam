# ROS2 上层架构说明（Astra Pro + YDLIDAR X2 + STM32）

## 1. 目标

统一入口：`robot_bringup/launch/system.launch.py`

上层支持三件事：
- 建图（Mapping）
- 导航（Navigation）
- 底盘控制（STM32 串口桥）

并且建图源可切换：
- `mapping_source:=camera`（Astra 深度转激光）
- `mapping_source:=lidar`（YDLIDAR X2）

## 2. 当前建图链路

### 2.1 摄像头建图（camera 模式）

数据流：
- `astra_camera_node` 发布 `/camera/depth/image_raw` + `/camera/depth/camera_info`
- `depthimage_to_laserscan` 转成 `/camera/scan`
- `slam_toolbox` 订阅 `/camera/scan` 生成 `/map`

说明：
- 这个模式不依赖视觉特征点里程计。
- 在 `base_mode:=none` 时，会自动补 `odom -> base_link` 静态 TF，保证 TF 链完整。

### 2.2 雷达建图（lidar 模式）

数据流：
- `ydlidar_ros2_driver_node` 发布 `/scan`
- `slam_toolbox` 订阅 `/scan` 生成 `/map`

## 3. 启动命令

### 3.1 摄像头建图（当前优先）

```bash
source /opt/ros/foxy/setup.bash
source /home/robot/ros2_ws/install/setup.bash
ros2 launch robot_bringup system.launch.py \
  mode:=mapping \
  mapping_source:=camera \
  use_camera:=true \
  base_mode:=none \
  use_rviz:=true
```

### 3.2 雷达建图

```bash
source /opt/ros/foxy/setup.bash
source /home/robot/ros2_ws/install/setup.bash
ros2 launch robot_bringup system.launch.py \
  mode:=mapping \
  mapping_source:=lidar \
  use_camera:=false \
  base_mode:=none \
  use_rviz:=true
```

### 3.3 导航（接入底盘+地图后）

```bash
source /opt/ros/foxy/setup.bash
source /home/robot/ros2_ws/install/setup.bash
ros2 launch robot_bringup system.launch.py \
  mode:=navigation \
  base_mode:=real \
  map_file:=/home/robot/ros2_maps/my_map.yaml
```

### 3.4 导航（底盘未接入，虚拟底盘联调）

```bash
source /opt/ros/foxy/setup.bash
source /home/robot/ros2_ws/install/setup.bash
ros2 launch robot_bringup system.launch.py \
  mode:=navigation \
  base_mode:=fake \
  map_file:=/home/robot/ros2_maps/my_map.yaml
```

## 4. 一键脚本

`launch_scripts/start_mapping.sh` 支持切换：

```bash
# 摄像头建图
/home/robot/ros2_ws/launch_scripts/start_mapping.sh camera

# 雷达建图
/home/robot/ros2_ws/launch_scripts/start_mapping.sh lidar

# 雷达建图（高精档）
/home/robot/ros2_ws/launch_scripts/start_mapping.sh lidar quality

# 雷达建图（更高精度档）
/home/robot/ros2_ws/launch_scripts/start_mapping.sh lidar precision

# 雷达建图（无 RViz）
/home/robot/ros2_ws/launch_scripts/start_mapping.sh lidar precision --no-rviz
```

说明：
- `lidar` 模式默认走 `quality` 参数档位。
- 启动前会自动调用 `launch_scripts/check_lidar_health.sh` 做雷达数据验收。

## 5. 关键文件

- `src/robot_bringup/launch/system.launch.py`：总编排 + mapping_source 切换
- `src/robot_bringup/launch/camera_scan.launch.py`：深度图转激光
- `src/robot_bringup/launch/slam.launch.py`：slam_toolbox 启动与 scan topic 注入
- `src/ydlidar_ros2_driver/launch/astra_pro.launch.py`：Astra Pro 驱动与相机静态 TF
- `src/robot_bringup/config/slam_toolbox_mapping.yaml`：SLAM 参数

## 6. 快速检查

```bash
# 摄像头模式下应看到
ros2 topic list | grep -E "^/camera/depth/image_raw$|^/camera/scan$|^/map$"

# 雷达模式下应看到
ros2 topic list | grep -E "^/scan$|^/map$"
```

如果 `/camera/scan` 没有数据，优先检查：
- Astra 是否在发 `/camera/depth/image_raw`
- 深度图是否有有效范围（不是全黑/全 0）
- 相机 USB 供电是否稳定
