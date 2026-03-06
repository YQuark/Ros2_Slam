#!/usr/bin/env python3
# 简单的里程计发布器（用于测试建图）

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 发布静态odom（用于测试建图）
    # 注意：这只是测试用的静态odom，实际应用需要真实的里程计
    odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation time if true'),
        odom_publisher,
    ])
