# 上位机与下位机代码全面审计执行计划

## 内部执行等级

L：跨多个代码树的静态审计与文档生成，适合串行推进，不需要子代理。

## 阶段

1. 工作区和约束确认
   - 确认当前目录是挂载环境。
   - 遵守禁止本地构建、运行、ROS 检查和硬件访问的限制。

2. 静态代码清点
   - 清点自研上位机文件。
   - 清点 STM32 Core 与 ESP01S 文件。
   - 识别第三方代码和生成目录，避免把 vendor 代码当作自研代码误判。

3. 深度审计
   - 审计 ROS2 桥接节点、launch/config、启动脚本。
   - 审计 STM32 协议、运动控制、串口、看门狗、电池保护。
   - 审计 ESP01S Web 控制、Wi-Fi、HTTP 路由、串口协议。
   - 审计跨端协议一致性、速度量纲、里程计、TF、模式切换和安全控制。

4. 文档生成
   - 首先调用 WPS Word 连接能力。
   - 若 WPS Writer 未运行，则使用标准 OpenXML `.docx` 作为 Word 文档交付。
   - 输出到用户指定目录 `/mnt/d/document/projects`。

5. 收尾
   - 生成执行回执和清理回执。
   - 明确未执行本地验证，需在远程机器人环境验证。

## 验证策略

仅做静态验证：文件清点、代码阅读、关键模式搜索和证据行定位。

远程验证模板：

```bash
cd ~/ros2_ws
colcon build --packages-select stm32_robot_bridge robot_bringup
source install/setup.bash
ros2 launch robot_bringup system.launch.py mode:=mapping use_lidar:=true base_mode:=real
```

可选检查：

```bash
ros2 topic list
ros2 node list
ros2 topic echo /odom
ros2 topic echo /scan
```
