#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _compose(context):
    share = get_package_share_directory("robot_state_estimation")
    requested = LaunchConfiguration("base_fusion_mode").perform(context).strip().lower()
    aliases = {"none": "wheel", "ekf": "wheel_imu"}
    fusion_mode = aliases.get(requested, requested)
    if fusion_mode not in ("wheel", "wheel_imu"):
        raise RuntimeError("base_fusion_mode must be wheel or wheel_imu")
    effective_params = LaunchConfiguration("effective_params_file").perform(context).strip()
    node_params = [effective_params] if effective_params else []
    ekf_file = (
        LaunchConfiguration("ekf_wheel_params_file").perform(context)
        if fusion_mode == "wheel"
        else LaunchConfiguration("ekf_wheel_imu_params_file").perform(context)
    )
    if not os.path.isfile(ekf_file):
        raise RuntimeError(f"EKF params file does not exist: {ekf_file}")
    return [
        Node(
            package="robot_state_estimation",
            executable="wheel_odometry_node.py",
            name="wheel_odometry",
            output="screen",
            parameters=node_params,
        ),
        Node(
            package="robot_state_estimation",
            executable="imu_adapter_node.py",
            name="imu_adapter",
            output="screen",
            parameters=node_params,
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="base_ekf",
            output="screen",
            parameters=[ekf_file],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory("robot_state_estimation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("base_fusion_mode", default_value="wheel"),
            DeclareLaunchArgument("effective_params_file", default_value=""),
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument(
                "ekf_wheel_params_file",
                default_value=os.path.join(share, "config", "ekf_wheel.yaml"),
            ),
            DeclareLaunchArgument(
                "ekf_wheel_imu_params_file",
                default_value=os.path.join(share, "config", "ekf_base.yaml"),
            ),
            OpaqueFunction(function=_compose),
        ]
    )
