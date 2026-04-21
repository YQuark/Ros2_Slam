# robot_bringup

`robot_bringup` 是上位机的技术编排包，不负责运维入口聚合，只负责 launch、参数和可视化配置。

## 包职责

- 编排建图模式：`mode:=mapping`
- 编排导航模式：`mode:=navigation`
- 统一传感器层：雷达、相机、深度转激光、视觉里程计
- 统一底盘层：真实底盘、虚拟底盘、无底盘占位 TF
- 可选底盘融合层：`/odom + /imu/data -> /odometry/filtered`
- 统一可视化层：RViz 与地图点云兼容显示

## 核心入口

```bash
ros2 launch robot_bringup system.launch.py mode:=mapping
```

## 目录约定

- `launch/`
  只放技术 launch，不放运维脚本。
- `config/`
  只放参数与设备配置。
- `rviz/`
  只放 RViz 视图模板。
- `scripts/`
  只放随包安装的 ROS 节点脚本。

## 关键 launch

- `launch/system.launch.py`
  总编排入口。
- `launch/sensors.launch.py`
  传感器层。
- `launch/base.launch.py`
  真实底盘桥接。
- `launch/base_ekf.launch.py`
  底盘 EKF 融合层。
- `launch/slam.launch.py`
  `slam_toolbox` 建图。
- `launch/localization.launch.py`
  定位。
- `launch/nav2.launch.py`
  `Nav2`。

## 使用原则

- 用户入口优先使用 `/home/robot/ros2_ws/launch_scripts/robot.sh`
- 只有在二次开发或调试 launch 行为时，才直接调用本包 launch 文件
- 兼容参数 `use_base` 仍保留，但新配置应优先使用 `base_mode:=real|fake|none`
- 如果启用底盘融合，应优先使用 `base_fusion_mode:=ekf`

## 当前默认约定

- 雷达外参默认从 `base_link` 发布到 `laser_frame`：`x=0.07, y=0.0, z=0.13, roll=0, pitch=0, yaw=1.570796326795`
- `precision` 雷达建图档只保留更细的 `0.03m` 地图分辨率，其余匹配行为尽量贴近 `quality`
- 底盘命令时序默认和 `robot.sh navigation` 保持一致：`cmd_timeout=0.25s`、`drive_keepalive_sec=0.10s`
