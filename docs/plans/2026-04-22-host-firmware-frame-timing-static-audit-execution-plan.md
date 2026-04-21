# 上位机与下位机帧时序静态审计执行计划

## 内部执行等级

L：跨两个代码仓的静态审计和一致性修复，适合单通道串行执行。

## 执行步骤

1. 进行 skeleton check，确认 `Ros2_Slam` 与 `Clion` 的仓库位置、相关代码入口和当前未提交改动。
2. 冻结需求和计划文档，建立本轮 `vibe` 审计工件。
3. 审计上位机：
   - `robot.sh`
   - `system.launch.py`
   - `base.launch.py`
   - `stm32_bridge.launch.py`
   - `bridge_node.py`
   - `fake_base_odom.py`
4. 审计下位机：
   - `robot_config.h`
   - `main.c`
   - `robot_control.c`
   - `link_proto.c`
   - `pc_link.c`
   - `ESP01S.ino`
5. 建立静态时序表，核对：
   - host control/status 频率
   - host keepalive 与 timeout
   - MCU control period
   - MCU command timeout / link active timeout
   - ESP refresh / idle-stop
   - UART 发送队列和 backlog 保护
6. 修复静态可证的一致性问题，包括默认值、日志语义、文档口径和明确的代码缺陷。
7. 执行语法、配置、格式和文档一致性验证。
8. 产出最终静态审计结论，并明确哪些问题仍需实机确认。

## 验证策略

- `bash -n` 校验 shell 脚本
- `python3 -m py_compile` 校验 Python 文件
- YAML / XML 解析校验
- `git diff --check`
- `rg` 交叉扫描默认值、时序参数、文档口径和协议字段

## 完成语言规则

- 只能对静态证据已覆盖的部分说“已对齐”或“已修复”。
- 对运行态和实机相关问题，必须明确标记为“仍需实机验证”。

## 清理要求

- 写入本轮 `vibe` 工件：
  - `skeleton-receipt.json`
  - `intent-contract.json`
  - `phase-static-audit.json`
  - `cleanup-receipt.json`
- 不改写历史 `docs/requirements` 与 `docs/plans` 文档。
