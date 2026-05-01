# 系统架构索引

`SYSTEM_OVERVIEW.md` 作为仓库根目录入口保留。当前 ARM 迁移以 `AGENTS.md` 中的 Raspberry Pi 4B + ROS2 Humble + YDLIDAR X2 + STM32 两轮差速底盘为约束。

建议按以下顺序阅读：

1. [项目范围](./docs/00_project_scope.md)
2. [树莓派环境](./docs/01_raspberrypi_setup.md)
3. [硬件连接](./docs/02_hardware_connection.md)
4. [YDLIDAR X2 与建图入口](./docs/03_lidar_mapping.md)
5. [排障顺序](./docs/99_troubleshooting.md)

当前体系只有两层推荐入口：

1. 技术入口：`src/robot_bringup/launch/system.launch.py`
2. 运维入口：`launch_scripts/robot.sh`

如果你还在使用 `start_mapping.sh`、`start_navigation.sh` 等脚本，可以继续使用；它们只是兼容包装，不承载主逻辑。

当前仓库推荐按这条路径理解：

1. 先看根目录 [README.md](./README.md) 了解目录分层
2. 再看 [docs/02-系统架构.md](./docs/02-系统架构.md) 了解代码层级
3. 最后按 [docs/01-快速开始.md](./docs/01-快速开始.md) 或 [launch_scripts/README.md](./launch_scripts/README.md) 启动
