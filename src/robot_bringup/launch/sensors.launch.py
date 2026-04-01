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
        arguments=[
            LaunchConfiguration('lidar_tf_x'),
            LaunchConfiguration('lidar_tf_y'),
            LaunchConfiguration('lidar_tf_z'),
            LaunchConfiguration('lidar_tf_roll'),
            LaunchConfiguration('lidar_tf_pitch'),
            LaunchConfiguration('lidar_tf_yaw'),
            'base_link',
            'laser_frame',
        ],
        output='screen',
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ydlidar_share, 'launch', 'astra_pro.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_camera')),
        launch_arguments={
            'enable_color': LaunchConfiguration('camera_enable_color'),
            'enable_ir': LaunchConfiguration('camera_enable_ir'),
            'use_uvc_camera': LaunchConfiguration('camera_use_uvc'),
            'color_info_url': LaunchConfiguration('camera_color_info_url'),
            'ir_info_url': LaunchConfiguration('camera_ir_info_url'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('lidar_params_file', default_value=os.path.join(ydlidar_share, 'params', 'X2.yaml')),
        DeclareLaunchArgument('lidar_tf_x', default_value='0.07'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.13'),
        DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument('camera_enable_color', default_value='true'),
        DeclareLaunchArgument('camera_enable_ir', default_value='false'),
        DeclareLaunchArgument('camera_use_uvc', default_value='true'),
        DeclareLaunchArgument('camera_color_info_url', default_value=''),
        DeclareLaunchArgument('camera_ir_info_url', default_value=''),
        lidar_node,
        lidar_tf,
        camera_launch,
    ])
