# 串口互斥防护执行计划

## 内部等级

M：单脚本精准修复，附带静态检查和远程验证命令。

## 执行步骤

1. 审计 `robot.sh` 中雷达参数生成、健康检查、底盘串口解析的顺序。
2. 抽取统一函数，在已知雷达口后解析真实底盘口。
3. 将该函数应用到建图、导航、完整系统入口。
4. 保留显式 `--base-port` 的用户意图，但对冲突端口执行 fail-fast。
5. 仅执行本地静态语法检查，运行验证留给远程机器人。

## 验证计划

本地只允许执行：

```bash
bash -n launch_scripts/robot.sh
```

远程机器人验证：

```bash
cd ~/ros2_ws/launch_scripts
./robot.sh mapping lidar --real-base --ekf-base
```

成功前置条件是启动日志中的 `雷达串口` 和 `底盘串口` 必须不同。

## 回滚规则

如远程脚本误判 STM32 串口，应只回滚或调整 `resolve_base_port_avoiding_lidar` 与探测逻辑，不回滚此前 IMU、EKF、文档和手动控制相关修改。
