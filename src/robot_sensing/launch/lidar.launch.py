#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robot_sensing")

    lidar_node = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="log",
        condition=IfCondition(LaunchConfiguration("use_lidar")),
        emulate_tty=True,
        parameters=[LaunchConfiguration("lidar_params_file")],
        remappings=[
            ("scan", LaunchConfiguration("lidar_raw_scan_topic")),
        ],
    )

    scan_normalizer = Node(
        package="robot_sensing",
        executable="scan_normalizer.py",
        name="scan_normalizer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_lidar")),
        parameters=[{
            "input_topic": LaunchConfiguration("lidar_raw_scan_topic"),
            "output_topic": LaunchConfiguration("lidar_scan_topic"),
            "output_size": LaunchConfiguration("lidar_scan_output_size"),
            "angle_min": LaunchConfiguration("lidar_scan_angle_min"),
            "angle_max": LaunchConfiguration("lidar_scan_angle_max"),
            "status_log_interval_sec": LaunchConfiguration("lidar_scan_normalizer_log_interval_sec"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("lidar_params_file", default_value=os.path.join(share, "config", "ydlidar_x2.yaml")),
        DeclareLaunchArgument("lidar_raw_scan_topic", default_value="/scan_raw"),
        DeclareLaunchArgument("lidar_scan_topic", default_value="/scan"),
        DeclareLaunchArgument("lidar_scan_output_size", default_value="425"),
        DeclareLaunchArgument("lidar_scan_angle_min", default_value=str(-3.141592653589793)),
        DeclareLaunchArgument("lidar_scan_angle_max", default_value=str(3.141592653589793)),
        DeclareLaunchArgument("lidar_scan_normalizer_log_interval_sec", default_value="30.0"),
        lidar_node,
        scan_normalizer,
    ])
