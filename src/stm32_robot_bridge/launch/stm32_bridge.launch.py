#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='auto'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('max_linear', default_value='0.50'),
        DeclareLaunchArgument('max_angular', default_value='1.50'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        Node(
            package='stm32_robot_bridge',
            executable='bridge_node',
            name='stm32_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'max_linear': LaunchConfiguration('max_linear'),
                'max_angular': LaunchConfiguration('max_angular'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }],
        ),
    ])
