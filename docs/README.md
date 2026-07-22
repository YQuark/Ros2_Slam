# 文档总览

当前架构与规范按主题维护，不再用数字文件名表示权威顺序。

- [系统架构](./architecture/system-overview.md)
- [配置单一事实源](./architecture/configuration.md)
- [TF 所有权](./architecture/tf_ownership.md)
- [Upper protocol v3](./interfaces/upper-protocol-v3.md)
- [v0.4.0 发布与回滚](./releases/v0.4.0.md)
- [架构决策记录](./adr/)
- [Verification 结构](../verification/README.md)

旧编号文档在 v0.4.x 作为兼容入口保留；其中与 API v3、配置编译器或本页冲突的内容均视为历史说明。用户入口为 `bin/robot`，`launch_scripts/robot.sh` 仅保留一版转发兼容。
