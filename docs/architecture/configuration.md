# 配置单一事实源

正式配置位于 `src/robot_config/config`。`bin/robot` 每次启动将 platform、safety、hardware、calibration 和 profile 合并、跨参数校验，并在运行目录生成 `effective-config.yaml`、`ros-params.yaml` 和带 SHA-256 的 manifest。

代码默认值仅用于开发和错误可读性，不是发布事实源。release mode 禁止 `unsafe_development_mode`，并要求 final 标定。任何实验与验收报告必须记录 commit、硬件 revision 和 effective config hash。
