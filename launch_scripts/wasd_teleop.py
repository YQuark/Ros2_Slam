#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
WASD 键盘遥控

  w / s : 前进 / 后退
  a / d : 左转 / 右转
  x 或 空格 : 立即停止
  q / z : 增加 / 降低线速度上限
  e / c : 增加 / 降低角速度上限
  Ctrl+C : 退出
"""


class WasdTeleop(Node):
    def __init__(self) -> None:
        super().__init__('wasd_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_step = 0.10
        self.angular_step = 0.40
        self.linear_speed = 0.20
        self.angular_speed = 0.80
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.create_timer(0.1, self.publish_current)

    def publish_current(self) -> None:
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.publisher.publish(msg)

    def stop(self) -> None:
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.publish_current()

    def handle_key(self, key: str) -> None:
        if key == 'w':
            self.current_linear = self.linear_speed
            self.current_angular = 0.0
        elif key == 's':
            self.current_linear = -self.linear_speed
            self.current_angular = 0.0
        elif key == 'a':
            self.current_linear = 0.0
            self.current_angular = self.angular_speed
        elif key == 'd':
            self.current_linear = 0.0
            self.current_angular = -self.angular_speed
        elif key in ('x', ' '):
            self.stop()
            return
        elif key == 'q':
            self.linear_speed = min(self.linear_speed + self.linear_step, 1.0)
            self.print_speed()
            return
        elif key == 'z':
            self.linear_speed = max(self.linear_speed - self.linear_step, 0.05)
            self.print_speed()
            return
        elif key == 'e':
            self.angular_speed = min(self.angular_speed + self.angular_step, 3.0)
            self.print_speed()
            return
        elif key == 'c':
            self.angular_speed = max(self.angular_speed - self.angular_step, 0.20)
            self.print_speed()
            return
        else:
            return

        self.publish_current()

    def print_speed(self) -> None:
        sys.stdout.write(
            f"\r线速度上限: {self.linear_speed:.2f} m/s | 角速度上限: {self.angular_speed:.2f} rad/s      \n"
        )
        sys.stdout.flush()


def main() -> int:
    print(HELP)
    sys.stdout.write("等待按键...\n")
    sys.stdout.flush()

    rclpy.init()
    node = WasdTeleop()

    stdin_fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(stdin_fd)
    tty.setcbreak(stdin_fd)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue

            key = sys.stdin.read(1)
            if key == '\x03':
                break
            node.handle_key(key)
    finally:
        node.stop()
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        sys.stdout.write("\n已停止遥控并发送零速。\n")
        sys.stdout.flush()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
