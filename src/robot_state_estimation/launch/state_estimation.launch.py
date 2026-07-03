#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _compose(context):
    share = get_package_share_directory("robot_state_estimation")
    fusion_mode = LaunchConfiguration("base_fusion_mode").perform(context).strip().lower()
    ekf_params_file = LaunchConfiguration("ekf_params_file").perform(context)

    if fusion_mode == "ekf":
        if not os.path.isfile(ekf_params_file):
            raise RuntimeError(f"EKF params file does not exist: {ekf_params_file}")
        return [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="base_ekf",
                output="screen",
                parameters=[ekf_params_file],
                remappings=[
                    ("/odometry/filtered", LaunchConfiguration("odom_topic")),
                ],
            )
        ]
    if fusion_mode == "none":
        return [
            Node(
                package="robot_state_estimation",
                executable="wheel_odom_republisher.py",
                name="wheel_odom_republisher",
                output="screen",
                parameters=[{
                    "input_topic": LaunchConfiguration("wheel_odom_topic"),
                    "output_topic": LaunchConfiguration("odom_topic"),
                    "publish_tf": LaunchConfiguration("publish_tf"),
                }],
            )
        ]
    raise RuntimeError("Invalid 'base_fusion_mode'. Use 'none' or 'ekf'.")


def generate_launch_description():
    share = get_package_share_directory("robot_state_estimation")

    return LaunchDescription([
        DeclareLaunchArgument("base_fusion_mode", default_value="none"),
        DeclareLaunchArgument("wheel_odom_topic", default_value="/wheel/odom"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("ekf_params_file", default_value=os.path.join(share, "config", "ekf_base.yaml")),
        OpaqueFunction(function=_compose),
    ])
