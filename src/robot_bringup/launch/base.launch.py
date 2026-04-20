#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base_share = get_package_share_directory('stm32_robot_bridge')

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(base_share, 'launch', 'stm32_bridge.launch.py')),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'max_linear': LaunchConfiguration('max_linear'),
            'max_angular': LaunchConfiguration('max_angular'),
            'cmd_timeout': LaunchConfiguration('cmd_timeout'),
            'drive_keepalive_sec': LaunchConfiguration('drive_keepalive_sec'),
            'auto_closed_loop_on_cmd': LaunchConfiguration('auto_closed_loop_on_cmd'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'imu_enabled': LaunchConfiguration('imu_enabled'),
            'publish_imu': LaunchConfiguration('publish_imu'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'imu_frame_id': LaunchConfiguration('imu_frame_id'),
            'use_status_yaw': LaunchConfiguration('use_status_yaw'),
            'status_yaw_mode': LaunchConfiguration('status_yaw_mode'),
            'status_yaw_jump_reject_deg': LaunchConfiguration('status_yaw_jump_reject_deg'),
            'odom_feedback_source': LaunchConfiguration('odom_feedback_source'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_track_width': LaunchConfiguration('wheel_track_width'),
            'encoder_cpr': LaunchConfiguration('encoder_cpr'),
            'odom_linear_scale': LaunchConfiguration('odom_linear_scale'),
            'odom_angular_scale': LaunchConfiguration('odom_angular_scale'),
            'odom_angular_sign': LaunchConfiguration('odom_angular_sign'),
            'status_log_interval_sec': LaunchConfiguration('status_log_interval_sec'),
            'cmd_log_interval_sec': LaunchConfiguration('cmd_log_interval_sec'),
            'excluded_ports': LaunchConfiguration('excluded_ports'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('max_linear', default_value='1.20'),
        DeclareLaunchArgument('max_angular', default_value='19.27'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.30'),
        DeclareLaunchArgument('drive_keepalive_sec', default_value='0.20'),
        DeclareLaunchArgument('auto_closed_loop_on_cmd', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('imu_enabled', default_value='true'),
        DeclareLaunchArgument('publish_imu', default_value='false'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('imu_frame_id', default_value='base_link'),
        DeclareLaunchArgument('use_status_yaw', default_value='true'),
        DeclareLaunchArgument('status_yaw_mode', default_value='relative'),
        DeclareLaunchArgument('status_yaw_jump_reject_deg', default_value='25.0'),
        DeclareLaunchArgument('odom_feedback_source', default_value='status_twist'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0325'),
        DeclareLaunchArgument('wheel_track_width', default_value='0.1250'),
        DeclareLaunchArgument('encoder_cpr', default_value='2340.0'),
        DeclareLaunchArgument('odom_linear_scale', default_value='1.0'),
        DeclareLaunchArgument('odom_angular_scale', default_value='1.0'),
        DeclareLaunchArgument('odom_angular_sign', default_value='1.0'),
        DeclareLaunchArgument('status_log_interval_sec', default_value='0.0'),
        DeclareLaunchArgument('cmd_log_interval_sec', default_value='0.0'),
        DeclareLaunchArgument('excluded_ports', default_value=''),
        bridge_launch,
    ])
