# 文档总览

当前文档以“职责与所有权 → 接口与时序 → 启动与验证 → 发布证据”为顺序。
若编号兼容文档与架构/契约文档冲突，以代码、`src/robot_config/config` 和本页列出的
权威文档为准。

## 架构与所有权

- [系统架构](./architecture/system-overview.md)
- [上位机系统上下文](./architecture/upper_system_context.md)
- [包职责矩阵](./architecture/package_responsibility_matrix.md)
- [Topic 所有权](./architecture/topic_ownership_matrix.md)
- [TF 所有权](./architecture/tf-ownership.md)
- [运行拓扑](./architecture/runtime_topology.md)
- [配置单一事实源](./architecture/configuration.md)
- [要求追踪](./architecture/requirement_traceability.md)

## 接口与安全契约

- [Platform API 5](./interfaces/platform-api-5.md)
- [Host 命令](./contracts/host-command-contract.md)
- [整车命令职权](./contracts/vehicle-command-authority.md)
- [观测](./contracts/observation-contract.md)
- [固件控制事实](./contracts/firmware-control-contract.md)
- [双层 rearm](./contracts/rearm-contract.md)
- [时序预算](./contracts/timing_budget.md)
- [配置所有权](./contracts/configuration-ownership.md)
- [电机布局](./contracts/motor_layout_contract.md)
- [底盘操作结果](./contracts/operation-result-contract.md)
- [TF 所有权契约](./contracts/tf-ownership.md)
- [Upper Protocol v3](./interfaces/upper-protocol-v3.md)

## 使用、验证与发布

- [快速开始](./01-快速开始.md)
- [建图指南](./04-建图指南.md)
- [建图测试流程](./05-建图测试流程.md)
- [导航拆解调试](./05b-导航拆解调试.md)
- [底盘与串口桥接](./06-底盘与串口桥接.md)
- [运维与排障](./07-运维与排障.md)
- [开发与二次复用](./08-开发与二次复用.md)
- [迁移与兼容](./09-迁移与兼容说明.md)
- [Verification 结构](../verification/README.md)
- [v0.6.0-rc2 状态与回滚](./releases/v0.6.0-rc2.md)

`docs/archive/` 和旧 release 文档只描述历史版本，不用于判断当前兼容性。
当前兼容事实只读取 `compatibility/firmware.yaml`；所有物理报告未 PASS 前，
`release_compatible` 必须保持 `false`。
