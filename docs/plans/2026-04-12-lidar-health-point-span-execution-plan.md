# 雷达健康检查点数波动执行计划

## 内部等级

M：单脚本判据修复。

## 执行步骤

1. 保留现有频率、消息数、CheckSum 检查。
2. 在采样脚本中输出点数最小值、最大值、绝对跨度和相对跨度。
3. 将失败条件从“档位数大于 1”改为“跨度过大且相对跨度过大”。
4. 对轻微波动输出提示，允许继续建图。

## 验证计划

本地只执行语法检查：

```bash
bash -n launch_scripts/check_lidar_health.sh
```

远程机器人验证：

```bash
cd ~/ros2_ws/launch_scripts
./robot.sh mapping lidar --real-base --ekf-base
```

## 完成语言约束

本地不能声明硬件验证通过，只能说明静态检查结果和远程验证命令。
