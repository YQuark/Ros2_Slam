# 固件协议黄金向量

`upper_v3_golden.json` 是 v0.4.0 当前规范向量，覆盖 HELLO、STATUS、IMU、
DIAGNOSTIC 与 velocity command。其 `source` 保持
`host-contract-unverified-by-firmware`，直到兼容固件仓库提供逐字节相同的
`tests/fixtures/upper_v3_golden.json`；届时 CI 通过 `FIRMWARE_CONTRACT_DIR`
执行双向比较并更新兼容矩阵，但不能提前把 HIL 标记为 PASS。

`upper_v2_golden.json` 只保留为 beta4 历史回归，不属于 v0.4.0 runtime
或发布验证入口。
