#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeBaseOdom(Node):
    def __init__(self):
        super().__init__('fake_base_odom')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('cmd_timeout', 0.25)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 20)
        self.create_timer(1.0 / max(self.publish_hz, 1.0), self._on_timer)

        self.get_logger().info(
            f'fake_base_odom started: {self.cmd_vel_topic} -> {self.odom_topic}, '
            f'hz={self.publish_hz:.1f} cmd_timeout={self.cmd_timeout:.2f}s'
        )

    def _on_cmd_vel(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.wz = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_update_time = now

        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            vx = 0.0
            wz = 0.0
        else:
            vx = self.vx
            wz = self.wz

        yaw_mid = self.yaw + 0.5 * wz * dt
        self.x += vx * math.cos(yaw_mid) * dt
        self.y += vx * math.sin(yaw_mid) * dt
        self.yaw += wz * dt

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.08
        odom.pose.covariance[7] = 0.08
        odom.pose.covariance[35] = 0.15
        odom.twist.covariance[0] = 0.08
        odom.twist.covariance[35] = 0.15
        self.odom_pub.publish(odom)

        if self.tf_br is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = self.child_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_br.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = FakeBaseOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
