# 兼容启动与运维脚本

`launch_scripts/` 是旧 CLI 的兼容层和现场诊断集合，不是配置事实源。
正式入口是仓库根目录的 `bin/robot`：它先从
`src/robot_config/config` 编译和校验 effective config，然后进入本目录的
`robot.sh`。不要直接编辑运行目录中的生成 YAML。

## 当前平台边界

- 正式 bringup 模式只有 `mapping` 和 `navigation`。
- `mapping` 默认不启用底盘；需显式选择 `--fake-base` 或
  `--real-base`。
- `navigation` 默认真实底盘，但当前 `release_compatible: false`，
  未通过 Gate B/C 时不得进行车轮落地运动。
- 默认估计是 wheel-only。`--ekf-base` 是兼容别名，映射到实验
  `wheel_imu`，不是正式默认。
- `--auto` / `--auto-drive` 虽仍被兼容 CLI 解析，但
  `system.launch.py` 会拒绝 `auto_mapping_drive=true`。frontier explorer 只在
  `experiments/legacy`。
- 底盘正式默认为 `/dev/serial0`；`auto` 已废弃。雷达默认为
  `/dev/ydlidar`。

## 安全起步

```bash
cd /home/robot/ros2_ws
./bin/robot --help

# 无真实底盘的建图拓扑联调
./bin/robot mapping lidar --manual --fake-base --rviz

# 假底盘导航拓扑联调，map 是位置参数
./bin/robot navigation /path/to/map.yaml --fake-base --rviz
```

真实底盘命令只用于已满足对应阶段门禁的环境：

```bash
./bin/robot mapping lidar --manual --real-base --base-port /dev/serial0 --rviz
./bin/robot navigation /path/to/map.yaml --real-base --base-port /dev/serial0 --rviz
```

## 诊断和维护

```bash
./bin/robot doctor
./bin/robot check lidar
./bin/robot check chassis
./bin/robot check tf
./bin/robot check mapping
./bin/robot check navigation
./bin/robot save-map my_map
./bin/robot stop
```

`check_*.sh`、`detect_*.sh`、`test_base_cmd.sh` 等可用于受控现场诊断，
但它们的成功不等价于 UART HIL、车辆或发布报告 PASS。
证据的结构和门禁见 [`verification/README.md`](../verification/README.md)。

## 所有权

- `bin/robot`：用户意图、配置编译和运行目录创建。
- `robot_bringup/system.launch.py`：只编排节点，不拥有算法状态。
- `robot_config`：配置 schema、来源和 artifact hash。
- `robot_verification`：Bag、HIL、标定和发布证据。

具体话题、TF 和 Action 所有权见
[`docs/architecture/topic_ownership_matrix.md`](../docs/architecture/topic_ownership_matrix.md)。
