# v0.3.0 rosbag 基准集

数据存放在 `/home/robot/robot_data/benchmarks/v0.3.0`，不提交 Git。数据集定义、最短时长、重复次数和统一话题清单位于 `config/benchmarks/rosbag_datasets.yaml`。

录制前应启动完整实车链路并确认 `/diagnostics` 为 OK。示例：

```bash
python3 tools/datasets/record_benchmark_bag.py bag_04_square
python3 tools/datasets/validate_benchmark_bag.py bag_04_square \
  /home/robot/robot_data/benchmarks/v0.3.0/bag_04_square/<run-id>
```

录制工具会在 bag 同级生成 manifest，锁定上位机提交、工作区脏状态、标定文件摘要、操作流程和实际命令。验收工具严格检查时长、全部必需话题以及非零消息数；任何一项缺失都不得进入 SLAM/Nav2 参数结论。
