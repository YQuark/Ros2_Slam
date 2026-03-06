#!/usr/bin/env python3
# ============================================
# Astra Pro 摄像头启动文件
# 使用官方 astra_camera 的 astra_pro.launch.xml
# ============================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    astra_share = get_package_share_directory('astra_camera')
    astra_xml = os.path.join(astra_share, 'launch', 'astra_pro.launch.xml')

    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra_xml),
        launch_arguments={
            'camera_name': 'camera',
            'vendor_id': '0x2bc5',
            'product_id': '0x0501',
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_depth': 'true',
            'enable_ir': LaunchConfiguration('enable_ir'),
            'publish_tf': 'false',
            'color_depth_synchronization': 'true',
            'use_uvc_camera': LaunchConfiguration('use_uvc_camera'),
            'uvc_vendor_id': '0x2bc5',
            'uvc_product_id': '0x0501',
            'uvc_camera_format': 'yuyv',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '15',
            'depth_width': '640',
            'depth_height': '480',
            # Match depth fps to color fps to improve RGB-D sync stability.
            'depth_fps': '15',
            'color_info_url': LaunchConfiguration('color_info_url'),
            'ir_info_url': LaunchConfiguration('ir_info_url'),
        }.items(),
    )

    # 外参：base_link -> camera_link（相机安装位姿）
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen',
    )

    camera_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera_depth_frame'],
        output='screen',
    )

    camera_ir_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_ir_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera_ir_frame'],
        output='screen',
    )

    camera_color_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera_color_frame'],
        output='screen',
    )

    camera_depth_to_color_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_to_color_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_depth_frame', 'camera_color_frame'],
        output='screen',
    )

    camera_color_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_optical_tf',
        arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'camera_color_frame', 'camera_color_optical_frame'],
        output='screen',
    )

    camera_depth_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_optical_tf',
        arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'camera_depth_frame', 'camera_depth_optical_frame'],
        output='screen',
    )

    camera_ir_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_ir_optical_tf',
        arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'camera_ir_frame', 'camera_ir_optical_frame'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('enable_ir', default_value='false'),
        DeclareLaunchArgument('use_uvc_camera', default_value='true'),
        DeclareLaunchArgument('color_info_url', default_value=''),
        DeclareLaunchArgument('ir_info_url', default_value=''),
        astra_launch,
        camera_tf,
        camera_depth_tf,
        camera_ir_tf,
        camera_color_tf,
        camera_depth_to_color_tf,
        camera_color_optical_tf,
        camera_depth_optical_tf,
        camera_ir_optical_tf,
    ])
