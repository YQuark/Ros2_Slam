#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rb_share = get_package_share_directory('robot_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_params_file',
            default_value=os.path.join(rb_share, 'config', 'ekf_base.yaml'),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='base_ekf',
            output='screen',
            parameters=[LaunchConfiguration('ekf_params_file')],
        ),
    ])
