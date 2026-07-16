#!/usr/bin/env python3
import secrets
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from robot_interfaces.msg import ChassisCommand

from robot_control.control_policy import (
    Command,
    CommandMux,
    InvalidCommandError,
    MotionLimiter,
    SelectedCommand,
    SourceConfig,
)


class CmdVelMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")

        self.declare_parameter("config_sha256", "development-uncompiled")
        self.declare_parameter("unsafe_development_mode", True)
        self.declare_parameter("research_sources", [])
        self.declare_parameter("linear_limit", 0.4)
        self.declare_parameter("angular_limit", 1.5)
        self.declare_parameter("input_linear_abs_max", 5.0)
        self.declare_parameter("input_angular_abs_max", 20.0)
        self.declare_parameter("max_linear_accel", 0.5)
        self.declare_parameter("max_angular_accel", 1.5)
        self.declare_parameter("max_linear_jerk", 2.0)
        self.declare_parameter("max_angular_jerk", 6.0)
        self.declare_parameter("motion_limiter_max_dt", 0.1)
        self.declare_parameter("timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("chassis_command_topic", "chassis/command")
        self.declare_parameter("publish_legacy_twist", False)
        self.declare_parameter("legacy_driver_topic", "cmd_vel/driver")

        research_sources = [str(value) for value in self.get_parameter("research_sources").value]
        self.source_config = SourceConfig(research_sources=research_sources)
        self.mux = CommandMux(
            source_config=self.source_config,
            linear_limit=float(self.get_parameter("linear_limit").value),
            angular_limit=float(self.get_parameter("angular_limit").value),
            timeout_sec=float(self.get_parameter("timeout_sec").value),
            input_linear_abs_max=float(self.get_parameter("input_linear_abs_max").value),
            input_angular_abs_max=float(self.get_parameter("input_angular_abs_max").value),
        )

        self.chassis_command_topic = str(self.get_parameter("chassis_command_topic").value)
        self.publisher = self.create_publisher(ChassisCommand, self.chassis_command_topic, 10)
        self.publish_legacy_twist = bool(self.get_parameter("publish_legacy_twist").value)
        self.legacy_driver_topic = str(self.get_parameter("legacy_driver_topic").value)
        self.legacy_publisher = (
            self.create_publisher(Twist, self.legacy_driver_topic, 10)
            if self.publish_legacy_twist
            else None
        )
        self.sequence = 0
        self.session_id = 0
        self.motion_limiter = MotionLimiter(
            max_linear_accel=float(self.get_parameter("max_linear_accel").value),
            max_angular_accel=float(self.get_parameter("max_angular_accel").value),
            max_linear_jerk=float(self.get_parameter("max_linear_jerk").value),
            max_angular_jerk=float(self.get_parameter("max_angular_jerk").value),
            max_dt_sec=float(self.get_parameter("motion_limiter_max_dt").value),
            max_linear_velocity=float(self.get_parameter("linear_limit").value),
            max_angular_velocity=float(self.get_parameter("angular_limit").value),
        )
        self.last_active = False
        self.last_source = "idle"
        self.invalid_command_count = 0
        self.subscriptions_by_source: Dict[str, object] = {}
        for source, topic in self.source_config.topic_map().items():
            self.subscriptions_by_source[source] = self.create_subscription(
                Twist,
                topic,
                self._make_callback(source),
                10,
            )

        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(1.0 / publish_hz, self._publish_selected)
        self.get_logger().info(
            "cmd_vel_mux active: "
            f"sources={','.join(self.source_config.topic_map().keys())} "
            f"chassis_command_topic={self.chassis_command_topic} "
            f"legacy_twist={1 if self.publish_legacy_twist else 0}"
        )

    def _make_callback(self, source: str):
        def _callback(msg: Twist) -> None:
            try:
                self.mux.update(
                    source,
                    Command(linear_x=msg.linear.x, angular_z=msg.angular.z),
                    self.get_clock().now().nanoseconds * 1e-9,
                )
            except (InvalidCommandError, TypeError, ValueError) as exc:
                self.invalid_command_count += 1
                self.get_logger().error(f"Rejected {source} command: {exc}")
                self._publish_release()
                return

            if not self.last_active:
                self._publish_selected()

        return _callback

    def _publish_selected(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        selected = self.mux.select(now_sec)
        if not selected.active:
            if self.last_active:
                self._publish_release()
            return

        if not self.last_active:
            self.session_id = secrets.randbits(64) or 1
            self.sequence = 0

        limited = self.motion_limiter.limit(selected.command, now_sec=now_sec)
        self._publish_command(SelectedCommand(source=selected.source, command=limited, active=True))
        self.last_active = True
        self.last_source = selected.source

    def _publish_release(self) -> None:
        if self.session_id == 0:
            self.session_id = secrets.randbits(64) or 1
        self.motion_limiter.reset()
        selected = SelectedCommand(source="idle", command=Command(0.0, 0.0), active=False)
        self._publish_command(selected)
        self.last_active = False
        self.last_source = "idle"

    def _publish_command(self, selected) -> None:
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        msg = ChassisCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = selected.command.linear_x
        msg.twist.angular.z = selected.command.angular_z
        msg.enable = selected.active
        msg.source = self._source_id(selected.source)
        msg.session_id = self.session_id
        msg.sequence = self.sequence
        self.publisher.publish(msg)

        if self.legacy_publisher is not None:
            legacy = Twist()
            legacy.linear.x = selected.command.linear_x
            legacy.angular.z = selected.command.angular_z
            self.legacy_publisher.publish(legacy)

    @staticmethod
    def _source_id(source: str) -> int:
        if source == "teleop":
            return ChassisCommand.SOURCE_TELEOP
        if source == "test":
            return ChassisCommand.SOURCE_TEST
        if source == "nav":
            return ChassisCommand.SOURCE_NAV
        if source.startswith("research/"):
            return ChassisCommand.SOURCE_RESEARCH
        return ChassisCommand.SOURCE_NONE


def main() -> None:
    rclpy.init()
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
