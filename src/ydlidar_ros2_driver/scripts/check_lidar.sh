#!/bin/bash

echo "=== YDLIDAR 激光雷达诊断工具 ==="
echo ""

# 检查 USB 设备
echo "1. 检查 USB 串口设备:"
ls -la /dev/ttyUSB* 2>/dev/null || echo "  未找到 /dev/ttyUSB 设备"
ls -la /dev/ttyACM* 2>/dev/null || echo "  未找到 /dev/ttyACM 设备"
echo ""

# 检查串口权限
echo "2. 检查串口权限:"
if [ -e /dev/ttyUSB0 ]; then
    ls -l /dev/ttyUSB0
else
    echo "  /dev/ttyUSB0 不存在"
fi
echo ""

# 显示可用的配置文件
echo "3. 可用的激光雷达配置文件:"
for file in /home/robot/ros2_ws/src/ydlidar_ros2_driver/params/*.yaml; do
    echo "  - $(basename $file)"
done
echo ""

echo "4. 根据你的激光雷达型号选择配置:"
echo "  X2/X4 系列: 使用 X2.yaml (波特率 115200)"
echo "  G1/G2/G6 系列: 使用对应型号配置 (波特率 230400)"
echo "  TminiPro: 使用 TminiPro.yaml"
echo ""

echo "5. 建议的启动命令:"
echo "  对于 X2/X4:"
echo "    ros2 launch ydlidar_ros2_driver slam_mapping.launch.py params_file:=X2.yaml"
echo ""
echo "  对于 G 系列:"
echo "    ros2 launch ydlidar_ros2_driver slam_mapping.launch.py params_file:=G2.yaml"
echo ""

echo "6. 如果仍失败，尝试手动设置串口权限:"
echo "    sudo chmod 777 /dev/ttyUSB0"
echo "    或:"
echo "    sudo usermod -a -G dialout \$USER"
echo ""