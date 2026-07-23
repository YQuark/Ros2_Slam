#!/usr/bin/env python3
"""Publish formal odometry and TF only while the compatibility gate permits it."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import PlatformCompatibilityState
from tf2_ros import TransformBroadcaster


class FormalOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("formal_odometry")
        self.declare_parameter("input_topic", "odometry/filtered_internal")
        self.declare_parameter("output_topic", "odom")
        self.declare_parameter("compatibility_state_topic", "platform/compatibility_state")
        self.permitted = False
        self.permitted_session = 0
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        compatibility_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PlatformCompatibilityState,
            str(self.get_parameter("compatibility_state_topic").value),
            self._on_compatibility,
            compatibility_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._on_odometry,
            20,
        )

    def _on_compatibility(self, msg: PlatformCompatibilityState) -> None:
        self.permitted = bool(msg.permit_formal_odometry and msg.observation_ready)
        self.permitted_session = int(msg.observation_session_id) if self.permitted else 0

    def _on_odometry(self, msg: Odometry) -> None:
        if not self.permitted or self.permitted_session == 0:
            return
        self.publisher.publish(msg)
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormalOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
