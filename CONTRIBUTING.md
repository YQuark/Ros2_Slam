# Contributing

每个改动必须说明是否改变 Platform API、wire protocol、安全不变量、TF/topic 所有权或标定参数。修改这些边界时同步更新 ADR、兼容矩阵和测试。软件测试通过不代表 HIL/实车通过；没有机器证据时保持 NOT_RUN。

运行 `scripts/verify/verify_upper.sh`。发布前还必须完成 v3 固件兼容锁定、HIL、实车、final 标定与 vehicle dynamics，并通过 release gate。
