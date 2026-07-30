# 固件协议黄金向量

`upper_v3_golden.json` 是 `bc472cc` / `v1.0.0-rc1` 的逐字节 Upper-v3
规范向量，覆盖 HELLO、两驱/四驱/单轮异常 STATUS、DIAGNOSTIC 和 IMU。
CI 通过 `FIRMWARE_CONTRACT_DIR` 比较固件 fixture、读取固件机器 schema，
并运行固件 Host CTest；这些软件证据不能代替真实 UART HIL。

`upper_v2_golden.json` 只保留为 beta4 历史记录，不属于当前 runtime
或发布验证入口。
