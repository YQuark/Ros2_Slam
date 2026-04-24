#!/usr/bin/env python3

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class PhaseStats:
    label: str
    cmd_vx: float
    cmd_wz: float
    duration: float
    sample_count: int = 0
    start_x: Optional[float] = None
    start_y: Optional[float] = None
    start_yaw: Optional[float] = None
    end_x: Optional[float] = None
    end_y: Optional[float] = None
    end_yaw: Optional[float] = None
    peak_abs_vx: float = 0.0
    peak_abs_wz: float = 0.0

    def observe(self, x: float, y: float, yaw: float, vx: float, wz: float) -> None:
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw
        self.end_x = x
        self.end_y = y
        self.end_yaw = yaw
        self.sample_count += 1
        self.peak_abs_vx = max(self.peak_abs_vx, abs(vx))
        self.peak_abs_wz = max(self.peak_abs_wz, abs(wz))

    def summary_line(self) -> str:
        if self.sample_count == 0 or self.start_x is None or self.end_x is None:
            return (
                f"SUMMARY {self.label}: samples=0 cmd_vx={self.cmd_vx:.3f} "
                f"cmd_wz={self.cmd_wz:.3f} status=NO_ODOM"
            )
        dx = self.end_x - self.start_x
        dy = self.end_y - self.start_y
        dist = math.hypot(dx, dy)
        yaw_delta_deg = math.degrees(wrap_angle(self.end_yaw - self.start_yaw))
        return (
            f"SUMMARY {self.label}: samples={self.sample_count} "
            f"cmd_vx={self.cmd_vx:.3f} cmd_wz={self.cmd_wz:.3f} "
            f"dx={dx:.3f} dy={dy:.3f} dist={dist:.3f} "
            f"yaw_delta_deg={yaw_delta_deg:.1f} "
            f"peak_vx={self.peak_abs_vx:.3f} peak_wz_deg_s={math.degrees(self.peak_abs_wz):.1f}"
        )

    def yaw_delta_deg(self) -> Optional[float]:
        if self.sample_count == 0 or self.start_yaw is None or self.end_yaw is None:
            return None
        return math.degrees(wrap_angle(self.end_yaw - self.start_yaw))

    def effective_wz_deg_s(self) -> Optional[float]:
        yaw_delta = self.yaw_delta_deg()
        if yaw_delta is None or self.duration <= 0.0:
            return None
        return yaw_delta / self.duration


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
    parser.add_argument('--mode', choices=('rotate', 'linear', 'angular_sweep'), default='rotate')
    parser.add_argument('--linear', type=float, default=0.08)
    parser.add_argument('--angular', type=float, default=0.35)
    parser.add_argument('--angular-series', type=str, default='0.35,0.7,1.0,1.5,2.0,3.0')
    parser.add_argument('--duration', type=float, default=1.5)
    parser.add_argument('--hz', type=float, default=10.0)
    parser.add_argument('--wait-subscriber-sec', type=float, default=8.0)
    parser.add_argument('--odom-settle-sec', type=float, default=0.5)
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node('base_cmd_test_publisher')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    current_phase: Optional[PhaseStats] = None
    phases: List[PhaseStats] = []

    def on_odom(msg: Odometry) -> None:
        nonlocal current_phase
        if current_phase is None:
            return
        pose = msg.pose.pose
        twist = msg.twist.twist
        current_phase.observe(
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(float(pose.orientation.z), float(pose.orientation.w)),
            float(twist.linear.x),
            float(twist.angular.z),
        )

    node.create_subscription(Odometry, '/odom', on_odom, 20)

    def run_phase(label: str, vx: float, wz: float, duration: float) -> None:
        nonlocal current_phase
        current_phase = PhaseStats(label=label, cmd_vx=float(vx), cmd_wz=float(wz), duration=float(duration))
        publish_for(node, publisher, vx, wz, duration, args.hz, label)
        settle_deadline = time.monotonic() + max(args.odom_settle_sec, 0.0)
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        phases.append(current_phase)
        print(current_phase.summary_line(), flush=True)
        current_phase = None

    def print_calibration() -> None:
        for phase in phases:
            if phase.sample_count == 0 or abs(phase.cmd_wz) < 1e-6:
                continue
            effective = phase.effective_wz_deg_s()
            if effective is None:
                continue
            cmd_wz_deg_s = math.degrees(phase.cmd_wz)
            ratio = effective / cmd_wz_deg_s if abs(cmd_wz_deg_s) > 1e-6 else 0.0
            print(
                f"CALIBRATION {phase.label}: "
                f"cmd_wz_deg_s={cmd_wz_deg_s:.1f} effective_wz_deg_s={effective:.1f} "
                f"response_ratio={ratio:.3f}",
                flush=True,
            )

    if not wait_for_subscriber(node, publisher, args.wait_subscriber_sec):
        print('ERROR: no /cmd_vel subscriber found', file=sys.stderr, flush=True)
        publish_stop(node, publisher, args.hz, 0.5)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    try:
        run_phase('stop_before', 0.0, 0.0, 0.8)
        if args.mode == 'angular_sweep':
            angular_values = []
            for part in args.angular_series.split(','):
                part = part.strip()
                if not part:
                    continue
                angular_values.append(float(part))
            if not angular_values:
                print('ERROR: angular_sweep requires at least one angular-series value', file=sys.stderr, flush=True)
                return 3
            for value in angular_values:
                run_phase(f'rotate_left_{value:.2f}', 0.0, value, args.duration)
                run_phase(f'stop_after_left_{value:.2f}', 0.0, 0.0, 0.8)
                run_phase(f'rotate_right_{value:.2f}', 0.0, -value, args.duration)
                run_phase(f'stop_after_right_{value:.2f}', 0.0, 0.0, 0.8)
            print_calibration()
        else:
            run_phase('rotate_left', 0.0, args.angular, args.duration)
            run_phase('stop_mid', 0.0, 0.0, 0.8)
            run_phase('rotate_right', 0.0, -args.angular, args.duration)
            run_phase('stop_after_rotate', 0.0, 0.0, 0.8)
        if args.mode == 'linear':
            run_phase('forward', args.linear, 0.0, args.duration)
            run_phase('stop_mid_linear', 0.0, 0.0, 0.8)
            run_phase('backward', -args.linear, 0.0, args.duration)
            run_phase('stop_end', 0.0, 0.0, 0.8)
    finally:
        publish_stop(node, publisher, args.hz, 0.8)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
