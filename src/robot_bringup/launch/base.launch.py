#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base_share = get_package_share_directory('stm32_robot_bridge')

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(base_share, 'launch', 'stm32_bridge.launch.py')),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'max_linear': LaunchConfiguration('max_linear'),
            'max_angular': LaunchConfiguration('max_angular'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'imu_enabled': LaunchConfiguration('imu_enabled'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='auto'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('max_linear', default_value='0.50'),
        DeclareLaunchArgument('max_angular', default_value='1.50'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('imu_enabled', default_value='true'),
        bridge_launch,
    ])
