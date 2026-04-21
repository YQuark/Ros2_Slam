#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_share = get_package_share_directory('nav2_bringup')
    rb_share = get_package_share_directory('robot_bringup')
    default_bt_xml = os.path.join(rb_share, 'behavior_trees', 'navigate_to_pose_recovery.xml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(nav2_share, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('default_bt_xml_filename', default_value=default_bt_xml),
        DeclareLaunchArgument('map_subscribe_transient_local', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'default_bt_xml_filename': LaunchConfiguration('default_bt_xml_filename'),
                'map_subscribe_transient_local': LaunchConfiguration('map_subscribe_transient_local'),
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),
    ])
