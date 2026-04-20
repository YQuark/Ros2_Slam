#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist


def publish_for(node, publisher, vx, wz, duration, hz, label):
    period = 1.0 / max(hz, 1.0)
    deadline = time.monotonic() + max(duration, 0.0)
    msg = Twist()
    msg.linear.x = float(vx)
    msg.angular.z = float(wz)
    print(f'TEST {label}: vx={vx:.3f} wz={wz:.3f} duration={duration:.2f}s', flush=True)
    while rclpy.ok() and time.monotonic() < deadline:
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)


def publish_stop(node, publisher, hz, duration=0.8):
    publish_for(node, publisher, 0.0, 0.0, duration, hz, 'stop')


def wait_for_subscriber(node, publisher, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if publisher.get_subscription_count() > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return publisher.get_subscription_count() > 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='Publish deterministic base /cmd_vel test commands.')
    parser.add_argument('--mode', choices=('rotate', 'linear'), default='rotate')
    parser.add_argument('--linear', type=float, default=0.08)
    parser.add_argument('--angular', type=float, default=0.35)
    parser.add_argument('--duration', type=float, default=1.5)
    parser.add_argument('--hz', type=float, default=10.0)
    parser.add_argument('--wait-subscriber-sec', type=float, default=8.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node('base_cmd_test_publisher')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    if not wait_for_subscriber(node, publisher, args.wait_subscriber_sec):
        print('ERROR: no /cmd_vel subscriber found', file=sys.stderr, flush=True)
        publish_stop(node, publisher, args.hz, 0.5)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    try:
        publish_stop(node, publisher, args.hz, 0.8)
        publish_for(node, publisher, 0.0, args.angular, args.duration, args.hz, 'rotate_left')
        publish_stop(node, publisher, args.hz, 0.8)
        publish_for(node, publisher, 0.0, -args.angular, args.duration, args.hz, 'rotate_right')
        publish_stop(node, publisher, args.hz, 0.8)
        if args.mode == 'linear':
            publish_for(node, publisher, args.linear, 0.0, args.duration, args.hz, 'forward')
            publish_stop(node, publisher, args.hz, 0.8)
            publish_for(node, publisher, -args.linear, 0.0, args.duration, args.hz, 'backward')
            publish_stop(node, publisher, args.hz, 0.8)
    finally:
        publish_stop(node, publisher, args.hz, 0.8)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
