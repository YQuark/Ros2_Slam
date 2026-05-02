#!/bin/bash

# 保存 slam_toolbox 建图结果的脚本
# 使用方法: ./save_map.sh [地图名称]

MAP_NAME=${1:-my_map}
MAP_DIR="$HOME/maps"

# 创建地图保存目录
mkdir -p "$MAP_DIR"

echo "正在保存地图到: $MAP_DIR/$MAP_NAME"
echo ""

# 使用 ros2 服务保存地图
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$MAP_DIR/$MAP_NAME'}}"

echo ""
echo "地图已保存！"
echo "地图文件:"
echo "  - $MAP_DIR/${MAP_NAME}.posegraph"
echo ""
echo "你可以通过以下命令继续编辑此地图:"
echo "ros2 launch ydlidar_ros2_driver slam_mapping.launch.py map_file_name:=$MAP_DIR/$MAP_NAME"