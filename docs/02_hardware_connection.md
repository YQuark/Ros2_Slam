# 硬件连接

YDLIDAR X2：

- 正式设备名：`/dev/ydlidar`
- 默认波特率：`115200`
- 默认 frame：`laser_frame`
- 参数文件：`src/robot_bringup/config/ydlidar_x2.yaml`

STM32 底盘：

- 正式设备名：`/dev/stm32_chassis`
- 默认波特率：`115200`
- 当前阶段不得用 `/dev/ttyUSB0` 顺序作为正式配置

udev 规则模板：

```bash
sudo ./scripts/setup_udev_rules.sh
```

模板中的 `idVendor` / `idProduct` 标为“待实测”，部署时必须用现场设备的 `udevadm info` 结果替换。
