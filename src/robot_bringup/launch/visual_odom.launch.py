#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('base_frame'),
                'odom_frame_id': LaunchConfiguration('odom_frame'),
                'publish_tf': True,
                'approx_sync': True,
                # Astra color/depth timestamps can drift by >50 ms, widen sync window.
                'approx_sync_max_interval': 0.10,
                'queue_size': 60,
                # Prefer reliable transport for local camera topics to reduce frame drops.
                'qos': 1,
                'qos_camera_info': 1,
                'publish_null_when_lost': True,
                'Odom/GuessMotion': 'true',
                'Odom/MinInliers': '8',
                'Vis/MinInliers': '6',
                'wait_for_transform': 1.0,
            }],
            remappings=[
                ('rgb/image', LaunchConfiguration('rgb_topic')),
                ('depth/image', LaunchConfiguration('depth_topic')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
            ],
        ),
    ])
