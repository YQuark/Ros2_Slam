#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')

    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        emulate_tty=True,
        parameters=[LaunchConfiguration('lidar_params_file')],
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
        output='screen',
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ydlidar_share, 'launch', 'astra_pro.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('lidar_params_file', default_value=os.path.join(ydlidar_share, 'params', 'X2.yaml')),
        lidar_node,
        lidar_tf,
        camera_launch,
    ])
