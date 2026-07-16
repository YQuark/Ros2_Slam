#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _compose(context):
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    allow_uncompiled = (
        LaunchConfiguration("allow_uncompiled_config").perform(context).lower() == "true"
    )
    parameters = []
    if params_file:
        if not os.path.isfile(params_file):
            raise RuntimeError(f"effective ROS params file does not exist: {params_file}")
        parameters.append(params_file)
    elif not allow_uncompiled:
        raise RuntimeError("effective params are required; start through bin/robot")
    parameters.append({"port": LaunchConfiguration("port")})
    return [
        Node(
            package="stm32_robot_bridge",
            executable="bridge_node",
            name="stm32_bridge",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=os.environ.get("ROBOT_EFFECTIVE_PARAMS", "")
            ),
            DeclareLaunchArgument("port", default_value="/dev/serial0"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("allow_uncompiled_config", default_value="false"),
            OpaqueFunction(function=_compose),
        ]
    )
