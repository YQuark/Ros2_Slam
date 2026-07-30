# 配置单一事实源

## 输入与产物

唯一手写事实源位于 `src/robot_config/config`：

```text
platform.yaml + safety.yaml + hardware/<robot>.yaml
+ calibration/<robot>/{chassis,imu,lidar}.yaml
+ profiles/<mode>.yaml + components/{lidar,slam,nav2,ekf}
```

`bin/robot` 每次启动生成独立 run 目录，编译器输出：

- `effective-config.yaml`
- `ros-params.yaml`
- lidar、SLAM、Nav2、wheel/EKF 参数
- launch 标定参数
- 记录输入/产物 SHA-256、字段 provenance、Git 身份和安全覆盖审计的 manifest

节点只消费编译产物；包内默认值只用于开发期报错可读性，不是发布参数。

## 校验边界

编译器使用 JSON Schema Draft 7，以兼容 Ubuntu 22.04/ROS Humble 的
`python3-jsonschema 3.x`。它拒绝未知字段、重复 YAML key、NaN/Inf、非法类型和
不满足以下关系的配置：

- `publish period < deadline <= lifespan < mux lease`
- `keepalive < bridge timeout < firmware timeout`
- `ack timeout <= bridge timeout`
- soft 速度不超过 hard 速度
- hard `(vx,wz)` 能映射到合法单轮周速
- wheel mask 限于四位且左右各至少一轮
- 标定 bundle 几何与算法几何一致
- supervisor 阈值和 score 严格递增
- runtime mode 与所选 profile 一致
- 正式模式禁止 provisional calibration 和 unsafe development

输出先写同级临时目录、fsync 后原子替换；已有目标不会被覆盖。release mode 还要求
干净且可追溯的 Git 身份、final chassis/IMU/lidar 标定和非零参数 CRC。

## 所有权

上位机拥有 frame、话题、Host 安全时序、运动限制、估计与导航参数。固件拥有 PID、
电机方向、控制参数和物理安全策略。上位机只记录期望固件 commit、硬件 revision 和
参数 CRC，不能维护第二份固件参数真值。
