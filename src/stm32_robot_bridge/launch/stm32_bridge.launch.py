#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/serial0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel/driver'),
        DeclareLaunchArgument('odom_topic', default_value='/wheel/odom'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.25'),
        DeclareLaunchArgument('drive_keepalive_sec', default_value='0.10'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('status_hz', default_value='100.0'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0350'),
        DeclareLaunchArgument('wheel_track_width', default_value='0.1760'),
        DeclareLaunchArgument('odom_linear_scale', default_value='1.0'),
        DeclareLaunchArgument('odom_angular_scale', default_value='1.0'),
        DeclareLaunchArgument('odom_angular_sign', default_value='1.0'),
        DeclareLaunchArgument('status_log_interval_sec', default_value='0.0'),
        DeclareLaunchArgument('cmd_log_interval_sec', default_value='0.0'),
        Node(
            package='stm32_robot_bridge',
            executable='bridge_node',
            name='stm32_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'cmd_timeout': LaunchConfiguration('cmd_timeout'),
                'drive_keepalive_sec': LaunchConfiguration('drive_keepalive_sec'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'status_hz': LaunchConfiguration('status_hz'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheel_track_width': LaunchConfiguration('wheel_track_width'),
                'odom_linear_scale': LaunchConfiguration('odom_linear_scale'),
                'odom_angular_scale': LaunchConfiguration('odom_angular_scale'),
                'odom_angular_sign': LaunchConfiguration('odom_angular_sign'),
                'status_log_interval_sec': LaunchConfiguration('status_log_interval_sec'),
                'cmd_log_interval_sec': LaunchConfiguration('cmd_log_interval_sec'),
            }],
        ),
    ])
