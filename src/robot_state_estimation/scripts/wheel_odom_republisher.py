#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class WheelOdomRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odom_republisher")
        self.declare_parameter("input_topic", "/wheel/odom")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.publisher = self.create_publisher(Odometry, str(self.get_parameter("output_topic").value), 20)
        self.tf_broadcaster = TransformBroadcaster(self) if bool(self.get_parameter("publish_tf").value) else None
        self.subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._on_odom,
            20,
        )
        self.get_logger().info(
            "wheel_odom_republisher active: "
            f"{self.get_parameter('input_topic').value} -> {self.get_parameter('output_topic').value}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self.publisher.publish(out)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header = out.header
            tf_msg.child_frame_id = out.child_frame_id
            tf_msg.transform.translation.x = out.pose.pose.position.x
            tf_msg.transform.translation.y = out.pose.pose.position.y
            tf_msg.transform.translation.z = out.pose.pose.position.z
            tf_msg.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = WheelOdomRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
