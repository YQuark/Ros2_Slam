# 上位机与下位机关联文档整理执行计划

## 内部等级

M：单轮文档整理，不需要子代理，不触碰构建或硬件。

## 步骤

1. 盘点 `docs/requirements`、`docs/plans`、`outputs/runtime/vibe-sessions`。
2. 识别旧中文专题文档文件名编码问题。
3. 新增稳定 ASCII 文件名的系统关联总览。
4. 更新 `docs/README.md`，把总览文档设为优先入口。
5. 写入本轮运行回执和清理回执。

## 验证边界

本地只做静态文档检查。远程构建、启动、topic/node/tf 检查不在本地执行。

## 回滚策略

如需回滚，仅删除 `docs/host-mcu-integration-status.md`、本轮需求/计划/回执，并还原 `docs/README.md`。
