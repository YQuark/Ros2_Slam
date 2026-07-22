#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=os.environ.get("ROBOT_EFFECTIVE_PARAMS", "")
            ),
            DeclareLaunchArgument("scenario", default_value="normal"),
            Node(
                package="robot_verification",
                executable="fake_base_node.py",
                name="fake_base",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"scenario": LaunchConfiguration("scenario")},
                ],
            ),
        ]
    )
