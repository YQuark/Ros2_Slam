# robot_bringup

`robot_bringup` 是上位机的技术编排包，不负责运维入口聚合，只负责 launch、参数和可视化配置。

## 包职责

- 编排建图模式：`mode:=mapping`
- 编排导航模式：`mode:=navigation`
- 统一传感器层：雷达、相机、深度转激光、视觉里程计
- 统一底盘层：真实底盘、虚拟底盘、无底盘占位 TF
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
