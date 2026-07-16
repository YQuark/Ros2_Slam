# Verification

`configs` 保存实验、HIL、benchmark 和 release 定义；`runners` 保存执行器；`reports` 只保存机器可读结果；`schemas` 保存格式约束。自动单元/组件/集成测试仍位于 `tests`。

旧的 `config/experiments`、`tools/experiments` 和 `reports` 路径在 v0.4.0 保留一版兼容入口，新工作不得再向旧目录添加结果。
