#!/usr/bin/env python3
# Legacy compatibility launch: redirect to robot_bringup mapping mode.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    ydlidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    bringup_share_dir = get_package_share_directory('robot_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='X2.yaml',
            description='Lidar params file name under ydlidar_ros2_driver/params',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share_dir, 'launch', 'system.launch.py')
            ),
            launch_arguments={
                'mode': 'mapping',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_camera': 'false',
                'use_base': 'true',
                'use_rviz': 'true',
                'lidar_params_file': PathJoinSubstitution(
                    [ydlidar_share_dir, 'params', LaunchConfiguration('params_file')]
                ),
            }.items(),
        ),
    ])
