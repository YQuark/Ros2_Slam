# Verification

`tests/` 保存自动单元、接口、协议、集成和部署契约测试；本目录保存需要数据、硬件或
发布身份的可追溯验证。

```text
verification/
├── configs/
│   ├── benchmarks/     # Bag/风险数据集清单
│   ├── experiments/    # EKF、SLAM、Nav2 实验定义
│   ├── hil/            # UART HIL 场景与阈值
│   └── release/        # 发布所需报告集合
├── runners/            # 可重复执行器
├── reports/            # 机器可读结果；模板初始为 NOT_RUN
└── schemas/            # effective config 与结果格式
```

## 证据顺序

```text
Host unit + protocol golden + PTY
  -> real UART HIL
  -> wheels-off-ground
  -> low-speed vehicle
  -> geometry/IMU/covariance calibration
  -> wheel-only versus EKF
  -> SLAM independent validation
  -> Nav2 10 goals x 5 runs
  -> fault injection + long run
```

每份 PASS 报告必须绑定相同的 upper tested commit、firmware compatible commit、
hardware revision、parameter CRC、effective config SHA-256、artifact SHA-256 manifest
和 calibration version，并包含实际测量 metrics。runner 使用严格递增事件时间和原子
报告写入；缺字段、零 CRC、非法 hash 或非递增时间都 fail-closed。

当前 rc2 报告仍为 `NOT_RUN`，不得因本地 pytest、Fake Base 或 PTY 通过而改写为
物理 PASS。发布门运行方式：

```bash
python3 scripts/verify/verify_release.py
```

在物理证据齐全前它应返回非零。旧 `config/`、`reports/` 路径仅为 v0.6.x 只读
兼容入口，新结果不得写入旧目录。
