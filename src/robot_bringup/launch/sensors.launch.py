#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')

    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        # The SDK can spam benign point-count warnings; keep them in ROS log files.
        output='log',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        emulate_tty=True,
        parameters=[LaunchConfiguration('lidar_params_file')],
        remappings=[
            ('scan', LaunchConfiguration('lidar_raw_scan_topic')),
        ],
    )

    scan_normalizer = Node(
        package='robot_bringup',
        executable='scan_normalizer.py',
        name='scan_normalizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        parameters=[{
            'input_topic': LaunchConfiguration('lidar_raw_scan_topic'),
            'output_topic': LaunchConfiguration('lidar_scan_topic'),
            'output_size': LaunchConfiguration('lidar_scan_output_size'),
            'angle_min': LaunchConfiguration('lidar_scan_angle_min'),
            'angle_max': LaunchConfiguration('lidar_scan_angle_max'),
            'status_log_interval_sec': LaunchConfiguration('lidar_scan_normalizer_log_interval_sec'),
        }],
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        arguments=[
            LaunchConfiguration('lidar_tf_x'),
            LaunchConfiguration('lidar_tf_y'),
            LaunchConfiguration('lidar_tf_z'),
            LaunchConfiguration('lidar_tf_yaw'),
            LaunchConfiguration('lidar_tf_pitch'),
            LaunchConfiguration('lidar_tf_roll'),
            'base_link',
            'laser_frame',
        ],
        output='screen',
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ydlidar_share, 'launch', 'astra_pro.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_camera')),
        launch_arguments={
            'enable_color': LaunchConfiguration('camera_enable_color'),
            'enable_ir': LaunchConfiguration('camera_enable_ir'),
            'use_uvc_camera': LaunchConfiguration('camera_use_uvc'),
            'color_info_url': LaunchConfiguration('camera_color_info_url'),
            'ir_info_url': LaunchConfiguration('camera_ir_info_url'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('lidar_params_file', default_value=os.path.join(ydlidar_share, 'params', 'X2.yaml')),
        DeclareLaunchArgument('lidar_raw_scan_topic', default_value='/scan_raw'),
        DeclareLaunchArgument('lidar_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('lidar_scan_output_size', default_value='425'),
        DeclareLaunchArgument('lidar_scan_angle_min', default_value=str(-3.141592653589793)),
        DeclareLaunchArgument('lidar_scan_angle_max', default_value=str(3.141592653589793)),
        DeclareLaunchArgument('lidar_scan_normalizer_log_interval_sec', default_value='30.0'),
        DeclareLaunchArgument('lidar_tf_x', default_value='0.07'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.13'),
        DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument(
            'lidar_tf_yaw',
            default_value='-1.570796326795',
            description='base_link -> laser_frame yaw in radians; override at runtime with lidar_tf_yaw:=<value>',
        ),
        DeclareLaunchArgument('camera_enable_color', default_value='true'),
        DeclareLaunchArgument('camera_enable_ir', default_value='false'),
        DeclareLaunchArgument('camera_use_uvc', default_value='true'),
        DeclareLaunchArgument('camera_color_info_url', default_value=''),
        DeclareLaunchArgument('camera_ir_info_url', default_value=''),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_lidar')),
            msg=[
                '[robot_bringup] lidar TF base_link -> laser_frame: ',
                'x=', LaunchConfiguration('lidar_tf_x'),
                ', y=', LaunchConfiguration('lidar_tf_y'),
                ', z=', LaunchConfiguration('lidar_tf_z'),
                ', yaw=', LaunchConfiguration('lidar_tf_yaw'),
                ' rad',
            ],
        ),
        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_lidar')),
            msg=[
                '[robot_bringup] lidar_tf_yaw is the runtime value. Override with lidar_tf_yaw:=<rad> when calibrating.',
            ],
        ),
        lidar_node,
        scan_normalizer,
        lidar_tf,
        camera_launch,
    ])
