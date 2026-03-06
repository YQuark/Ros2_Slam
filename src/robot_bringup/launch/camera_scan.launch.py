#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('scan_topic', default_value='/camera/scan'),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                # Keep scan frame consistent with depth image frame semantics.
                'output_frame': 'camera_depth_optical_frame',
                'scan_time': 0.1,
                # Use a small depth band instead of a single row to increase valid scan points.
                'scan_height': 12,
                'range_min': 0.20,
                'range_max': 8.0,
            }],
            remappings=[
                ('depth', LaunchConfiguration('depth_topic')),
                ('depth_camera_info', LaunchConfiguration('camera_info_topic')),
                ('scan', LaunchConfiguration('scan_topic')),
            ],
        ),
    ])
