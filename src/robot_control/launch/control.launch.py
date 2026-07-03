#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robot_control")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "cmd_vel_mux.yaml")),
        Node(
            package="robot_control",
            executable="cmd_vel_mux.py",
            name="cmd_vel_mux",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
