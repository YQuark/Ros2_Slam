# ADR 0005：配置事实源

决策：`robot_config` 的分层 YAML 是唯一手写事实源，启动只消费编译产物并记录 hash。安全不变量不可关闭；绕过只允许显式 unsafe development mode，release gate 必须拒绝。验证：schema、跨参数和 effective-config 测试。
