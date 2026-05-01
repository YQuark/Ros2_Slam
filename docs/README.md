# 文档总览

ARM 迁移优先阅读：

- [00_project_scope.md](./00_project_scope.md)
- [01_raspberrypi_setup.md](./01_raspberrypi_setup.md)
- [02_hardware_connection.md](./02_hardware_connection.md)
- [03_lidar_mapping.md](./03_lidar_mapping.md)
- [99_troubleshooting.md](./99_troubleshooting.md)

历史中文文档仍保留用于追溯旧 x86 工程能力，后续会按阶段迁移到标准文件名。

本目录是当前工程的主文档入口。代码是真值来源，用户侧唯一正式入口是 `launch_scripts/robot.sh`。

## 先看什么

推荐顺序：

1. [01-快速开始](./01-快速开始.md)
2. [02-系统架构](./02-系统架构.md)
3. [04-建图指南](./04-建图指南.md)
4. [07-运维与排障](./07-运维与排障.md)

如果你正在拆导航链、看恢复逻辑或查底盘闭环，再继续看：

- [05-导航拆解调试](./05-导航拆解调试.md)
- [06-底盘与串口桥接](./06-底盘与串口桥接.md)
- [host-mcu-integration-status](./host-mcu-integration-status.md)

## 当前入口约定

- 用户和运维入口：`launch_scripts/robot.sh`
- 工程编排入口：`src/robot_bringup/launch/system.launch.py`
- 默认导航流程：单阶段 `./robot.sh navigation ...`
- 两阶段导航：只作为回退和诊断模式

## 文档索引

- [01-快速开始](./01-快速开始.md)
- [02-系统架构](./02-系统架构.md)
- [03-硬件接线与设备识别](./03-硬件接线与设备识别.md)
- [04-建图指南](./04-建图指南.md)
- [05-建图测试流程](./05-建图测试流程.md)
- [05-导航拆解调试](./05-导航拆解调试.md)
- [06-底盘与串口桥接](./06-底盘与串口桥接.md)
- [07-运维与排障](./07-运维与排障.md)
- [08-开发与二次复用](./08-开发与二次复用.md)
- [09-迁移与兼容说明](./09-迁移与兼容说明.md)
- [上位机与下位机关联状态总览](./host-mcu-integration-status.md)
