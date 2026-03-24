# 系统架构索引

`SYSTEM_OVERVIEW.md` 作为仓库根目录兼容入口保留，完整内容已经迁移到 `docs/`。

建议按以下顺序阅读：

1. [快速开始](./docs/01-快速开始.md)
2. [系统架构](./docs/02-系统架构.md)
3. [建图指南](./docs/04-建图指南.md)
4. [导航指南](./docs/05-导航指南.md)
5. [运维与排障](./docs/07-运维与排障.md)

当前体系只有两层推荐入口：

1. 技术入口：`src/robot_bringup/launch/system.launch.py`
2. 运维入口：`launch_scripts/robot.sh`

如果你还在使用 `start_mapping.sh`、`start_navigation.sh` 等脚本，可以继续使用；它们现在只是兼容包装，不再承载主逻辑。
