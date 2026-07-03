#!/usr/bin/env python3
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from robot_control.control_policy import Command, CommandMux, SourceConfig


class CmdVelMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")

        self.declare_parameter("research_sources", [])
        self.declare_parameter("linear_limit", 0.4)
        self.declare_parameter("angular_limit", 1.5)
        self.declare_parameter("timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("driver_topic", "/cmd_vel/driver")

        research_sources = [str(value) for value in self.get_parameter("research_sources").value]
        self.source_config = SourceConfig(research_sources=research_sources)
        self.mux = CommandMux(
            source_config=self.source_config,
            linear_limit=float(self.get_parameter("linear_limit").value),
            angular_limit=float(self.get_parameter("angular_limit").value),
            timeout_sec=float(self.get_parameter("timeout_sec").value),
        )

        self.publisher = self.create_publisher(Twist, str(self.get_parameter("driver_topic").value), 10)
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
            f"driver_topic={self.get_parameter('driver_topic').value}"
        )

    def _make_callback(self, source: str):
        def _callback(msg: Twist) -> None:
            self.mux.update(
                source,
                Command(linear_x=float(msg.linear.x), angular_z=float(msg.angular.z)),
                self.get_clock().now().nanoseconds * 1e-9,
            )

        return _callback

    def _publish_selected(self) -> None:
        selected = self.mux.select(self.get_clock().now().nanoseconds * 1e-9)
        msg = Twist()
        msg.linear.x = selected.command.linear_x
        msg.angular.z = selected.command.angular_z
        self.publisher.publish(msg)


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
