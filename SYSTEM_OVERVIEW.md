# 系统架构索引

`SYSTEM_OVERVIEW.md` 作为仓库根目录入口保留。当前平台为 Raspberry Pi 4B + ROS2 Humble + YDLIDAR X2 + STM32 两轮差速底盘。

建议按以下顺序阅读：

1. [项目范围](./docs/00-项目范围.md) — 项目定位、平台约束、硬件清单
2. [快速开始](./docs/01-快速开始.md) — 构建、启动命令、预期状态
3. [系统架构](./docs/02-系统架构.md) — 分层、节点、话题、TF 树
4. [建图指南](./docs/04-建图指南.md) — 建图流程、SLAM 参数档
5. [运维与排障](./docs/07-运维与排障.md) — 日常检查、故障排查

当前体系只有两层推荐入口：

1. 技术入口：`src/robot_bringup/launch/system.launch.py`
2. 运维入口：`bin/robot`

如果你还在使用 `start_mapping.sh`、`start_navigation.sh` 等脚本，可以继续使用；它们只是兼容包装，不承载主逻辑。

当前仓库推荐按这条路径理解：

1. 先看根目录 [README.md](./README.md) 了解目录分层
2. 再看 [docs/02-系统架构.md](./docs/02-系统架构.md) 了解代码层级
3. 最后按 [docs/01-快速开始.md](./docs/01-快速开始.md) 或 [launch_scripts/README.md](./launch_scripts/README.md) 启动
