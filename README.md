# ROS2 机器人上位机工程

本仓库是 Raspberry Pi 4B 上位机工作区，目标是在 Ubuntu 22.04 64-bit + ROS2 Humble 下收口 YDLIDAR X2、STM32 两轮差速底盘、SLAM Toolbox 和 Nav2 主链路。

## 能力矩阵

| 能力 | 状态 | 说明 |
| --- | --- | --- |
| 雷达建图 | 已支持 | `YDLIDAR X2 -> /scan -> slam_toolbox` |
| 摄像头建图 | 后续扩展 | 摄像头默认不进入当前激光 SLAM 主链路 |
| 真实底盘导航 | 阻塞 | v0.4.0 只支持 upper v3；兼容固件、HIL 和实车验收尚未完成 |
| 虚拟底盘联调 | 已支持 | `base_mode:=fake` |
| 串口自动识别 | 已支持 | 自动区分底盘串口与雷达串口 |
| 底盘 EKF 融合 | 显式开关 | 默认使用 bridge odom，EKF 不默认启用 |
| 激光运行时覆写 | 已支持 | `mapping` / `navigation` 支持 `--lidar-reversion`、`--lidar-inverted`、`--lidar-yaw-*` |

## 统一入口

- 用户和运维入口：`/home/robot/ros2_ws/bin/robot`
- 工程编排入口：`/home/robot/ros2_ws/src/robot_bringup/launch/system.launch.py`

`launch_scripts/robot.sh` 在 v0.4.x 只做兼容转发。`bin/robot` 会先编译、校验并记录 effective config；`system.launch.py` 只用于开发、调试和二次编排。

## 快速路径

```bash
cd /home/robot/ros2_ws
source /opt/ros/humble/setup.bash
./scripts/bootstrap/fetch_vendor.sh
sudo ./scripts/bootstrap/install_ydlidar_sdk.sh
./scripts/build/build_ros_ws.sh
source /home/robot/ros2_ws/install/setup.bash
```

```bash
cd /home/robot/ros2_ws
./bin/robot mapping lidar --manual --real-base --no-rviz
./bin/robot save-map my_map
./bin/robot navigation --real-base --map my_map --no-rviz
```

标准导航流程只有一条：

1. 运行 `./bin/robot navigation --real-base --map my_map`
2. 在 RViz 中先执行 `2D Pose Estimate`
3. 等激光与地图基本重合后，再用 `2D Goal Pose` 下发目标

两阶段导航 `--localization-only` / `--nav2-only` 只保留为定位未就绪或现场异常时的回退流程，不再作为默认主流程。

## 当前默认约定

- 底盘正式串口：`/dev/stm32_chassis`
- 雷达正式串口：`/dev/ydlidar`
- 雷达参数文件：`src/robot_sensing/config/ydlidar_x2.yaml`
- 默认激光手性修正：`inverted: true`
- 雷达 yaw、轮径、轮距、编码器方向、电机方向：待实测
- 默认导航行为树：`src/robot_bringup/behavior_trees/navigate_to_pose_recovery.xml`
- 默认底盘命令时序：mux 250 ms、bridge 150 ms、keepalive 50 ms、固件 watchdog 200 ms
- 唯一配置事实源：`src/robot_config/config`；每次启动生成 config SHA-256
- Platform API / upper protocol：v3 only；beta4/v2 不兼容

## 仓库分层

```text
ros2_ws
├── deps/                  # 第三方依赖锁定清单与补丁
├── deploy/                # 树莓派部署模板
├── docs/                  # 标准阶段文档与旧中文文档
├── launch_scripts/        # 统一运维入口与诊断脚本
├── scripts/               # bootstrap / build / verify 脚本
├── verification/          # 机器实验、HIL、报告、schema 与发布门
├── tools/                 # 现场诊断和标定工具
├── compatibility/         # 固件兼容性声明
├── PLATFORM_API_VERSION
├── src/
│   ├── robot_description
│   ├── robot_config
│   ├── robot_sensing
│   ├── stm32_robot_bridge
│   ├── robot_state_estimation
│   ├── robot_control
│   ├── robot_bringup
│   └── vendor/            # vcs 自动拉取，Git 忽略
├── README.md
└── SYSTEM_OVERVIEW.md
```

运行数据不进仓库，树莓派上使用：

```text
/home/robot/robot_data/{maps,bags,reports,exports}
/home/robot/.config/slamrobot/robot.yaml
```

## 文档导航

- [文档总览](./docs/README.md)
- [项目范围](./docs/00-项目范围.md)
- [快速开始](./docs/01-快速开始.md)
- [系统架构](./docs/02-系统架构.md)
- [硬件接线与设备识别](./docs/03-硬件接线与设备识别.md)
- [Upper protocol v3](./docs/interfaces/upper-protocol-v3.md)
- [建图指南](./docs/04-建图指南.md)
- [建图测试流程](./docs/05-建图测试流程.md)
- [导航拆解调试](./docs/05b-导航拆解调试.md)
- [底盘与串口桥接](./docs/06-底盘与串口桥接.md)
- [运维与排障](./docs/07-运维与排障.md)
- [开发与二次复用](./docs/08-开发与二次复用.md)
- [迁移与兼容说明](./docs/09-迁移与兼容说明.md)
- [树莓派4B雷达迁移问题清单](./docs/10-树莓派4B雷达迁移问题清单.md)

## 设计原则

- 运维层只保留一个正式 CLI：`bin/robot`
- `start_*.sh` 继续存在，但只做兼容转发，不再维护独立逻辑
- 技术编排只保留一个核心入口：`src/robot_bringup/launch/system.launch.py`
- 第三方源码不提交；按 `deps/*.repos` 锁定 commit 后拉到忽略目录
- 对外平台接口固定为 `/scan`、`/odom`、`/tf`、`/tf_static` 和 `/cmd_vel/*` 候选输入
- 文档统一按“单阶段默认、两阶段回退”的导航语义描述
- 代码是真值来源，文档随代码更新
