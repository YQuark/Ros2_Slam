#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='auto'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.25'),
        DeclareLaunchArgument('drive_keepalive_sec', default_value='0.10'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_imu', default_value='false'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0325'),
        DeclareLaunchArgument('wheel_track_width', default_value='0.1250'),
        DeclareLaunchArgument('odom_linear_scale', default_value='1.0'),
        DeclareLaunchArgument('odom_angular_scale', default_value='1.0'),
        DeclareLaunchArgument('odom_angular_sign', default_value='1.0'),
        DeclareLaunchArgument('status_log_interval_sec', default_value='0.0'),
        DeclareLaunchArgument('cmd_log_interval_sec', default_value='0.0'),
        DeclareLaunchArgument('excluded_ports', default_value=''),
        Node(
            package='stm32_robot_bridge',
            executable='bridge_node',
            name='stm32_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'cmd_timeout': LaunchConfiguration('cmd_timeout'),
                'drive_keepalive_sec': LaunchConfiguration('drive_keepalive_sec'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'publish_imu': LaunchConfiguration('publish_imu'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'imu_frame_id': LaunchConfiguration('imu_frame_id'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheel_track_width': LaunchConfiguration('wheel_track_width'),
                'odom_linear_scale': LaunchConfiguration('odom_linear_scale'),
                'odom_angular_scale': LaunchConfiguration('odom_angular_scale'),
                'odom_angular_sign': LaunchConfiguration('odom_angular_sign'),
                'status_log_interval_sec': LaunchConfiguration('status_log_interval_sec'),
                'cmd_log_interval_sec': LaunchConfiguration('cmd_log_interval_sec'),
                'excluded_ports': LaunchConfiguration('excluded_ports'),
            }],
        ),
    ])
