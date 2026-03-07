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
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(rb_share, 'rviz', 'system.rviz')),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            additional_env={
                # Work around GLSL sampler-link errors on some Mesa/OGRE combinations.
                'LIBGL_ALWAYS_SOFTWARE': '1',
            },
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
