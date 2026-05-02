#!/usr/bin/env python3
"""
简单的里程计模拟器
使用键盘控制虚拟机器人移动，发布odom话题和TF
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import sys
import select
import termios
import tty


class OdomSimulator(Node):
    def __init__(self):
        super().__init__('odom_simulator')

        # 机器人位姿
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 当前速度
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # 上一次时间
        self.last_time = self.get_clock().now()

        # 发布者
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 读取键盘设置
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('=== 里程计模拟器已启动 ===')
        self.get_logger().info('使用 teleop_twist_keyboard 控制')
        self.get_logger().info('或在另一个终端: ros2 run teleop_twist_keyboard teleop_twist_keyboard')

        # 定时器：发布odom
        self.create_timer(0.05, self.update_odom)  # 20Hz

        # 定时器：显示状态
        self.create_timer(1.0, self.print_status)

    def cmd_vel_callback(self, msg):
        """接收速度命令"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def update_odom(self):
        """更新里程计"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt > 0:
            # 更新位姿
            self.x += self.linear_vel * math.cos(self.theta) * dt
            self.y += self.linear_vel * math.sin(self.theta) * dt
            self.theta += self.angular_vel * dt

            # 发布TF
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            # 四元数
            q = self.yaw_to_quaternion(self.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

            # 发布odom话题
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0

            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            odom.twist.twist.linear.x = self.linear_vel
            odom.twist.twist.angular.z = self.angular_vel

            self.odom_pub.publish(odom)

        self.last_time = current_time

    def yaw_to_quaternion(self, yaw):
        """将偏航角转换为四元数"""
        return [
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0)
        ]

    def print_status(self):
        """打印当前状态"""
        if abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01:
            self.get_logger().info(
                f'位置: x={self.x:.2f}, y={self.y:.2f}, theta={math.degrees(self.theta):.1f}° | '
                f'速度: v={self.linear_vel:.2f}, w={self.angular_vel:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    odom_sim = OdomSimulator()

    try:
        rclpy.spin(odom_sim)
    except KeyboardInterrupt:
        pass
    finally:
        odom_sim.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()