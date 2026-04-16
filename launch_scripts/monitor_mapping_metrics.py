#!/usr/bin/env python3

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def wrap_angle_deg(angle_deg: float) -> float:
    return math.degrees(math.atan2(math.sin(math.radians(angle_deg)), math.cos(math.radians(angle_deg))))


def yaw_deg_from_quat(z: float, w: float) -> float:
    return math.degrees(2.0 * math.atan2(z, w))


def finite_or_none(value: Optional[float]) -> str:
    if value is None:
        return '-'
    return f'{value:.3f}'


@dataclass
class OdomState:
    x: float
    y: float
    yaw_deg: float
    vx: float
    wz_deg_s: float


@dataclass
class ImuState:
    yaw_deg: Optional[float]
    gz_deg_s: float
    ax: float
    ay: float
    az: float
    yaw_valid: bool
    accel_valid: bool


class MappingMetricsMonitor(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('mapping_metrics_monitor')

        self.print_hz = max(args.print_hz, 0.2)
        self.static_linear_threshold = max(args.static_linear_threshold, 0.0)
        self.static_angular_threshold = max(args.static_angular_threshold, 0.0)
        self.static_imu_gz_threshold = max(args.static_imu_gz_threshold, 0.0)

        self.latest_odom: Optional[OdomState] = None
        self.latest_filtered: Optional[OdomState] = None
        self.latest_imu: Optional[ImuState] = None

        self.start_odom: Optional[OdomState] = None
        self.start_filtered: Optional[OdomState] = None
        self.start_imu_yaw_deg: Optional[float] = None

        self.static_active = False
        self.static_started_ns = 0
        self.static_odom_start: Optional[OdomState] = None
        self.static_filtered_start: Optional[OdomState] = None
        self.static_imu_start_yaw_deg: Optional[float] = None

        self.csv_file = None
        self.csv_writer = None
        if args.csv_out:
            csv_path = Path(args.csv_out).expanduser()
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = csv_path.open('w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'stamp_s',
                'odom_x', 'odom_y', 'odom_yaw_deg', 'odom_vx_mps', 'odom_wz_deg_s',
                'imu_yaw_deg', 'imu_gz_deg_s', 'imu_ax', 'imu_ay', 'imu_az',
                'filtered_x', 'filtered_y', 'filtered_yaw_deg', 'filtered_vx_mps', 'filtered_wz_deg_s',
                'odom_imu_yaw_diff_deg', 'filtered_imu_yaw_diff_deg', 'filtered_odom_yaw_diff_deg',
                'static_active', 'static_duration_s',
                'static_odom_drift_m', 'static_odom_yaw_drift_deg',
                'static_filtered_drift_m', 'static_filtered_yaw_drift_deg',
                'static_imu_yaw_drift_deg',
            ])

        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_filtered, 20)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 20)
        self.create_timer(1.0 / self.print_hz, self.on_timer)

        self.get_logger().info(
            'mapping_metrics_monitor started: '
            f'print_hz={self.print_hz:.1f} '
            f'static_linear_threshold={self.static_linear_threshold:.3f} '
            f'static_angular_threshold={self.static_angular_threshold:.3f} '
            f'static_imu_gz_threshold={self.static_imu_gz_threshold:.3f}'
        )
        if self.csv_writer is not None:
            self.get_logger().info(f'CSV output: {args.csv_out}')

    def on_odom(self, msg: Odometry) -> None:
        state = OdomState(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw_deg=wrap_angle_deg(yaw_deg_from_quat(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)),
            vx=float(msg.twist.twist.linear.x),
            wz_deg_s=math.degrees(float(msg.twist.twist.angular.z)),
        )
        self.latest_odom = state
        if self.start_odom is None:
            self.start_odom = state

    def on_filtered(self, msg: Odometry) -> None:
        state = OdomState(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw_deg=wrap_angle_deg(yaw_deg_from_quat(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)),
            vx=float(msg.twist.twist.linear.x),
            wz_deg_s=math.degrees(float(msg.twist.twist.angular.z)),
        )
        self.latest_filtered = state
        if self.start_filtered is None:
            self.start_filtered = state

    def on_imu(self, msg: Imu) -> None:
        yaw_cov = msg.orientation_covariance[8] if len(msg.orientation_covariance) >= 9 else 1e9
        accel_cov = msg.linear_acceleration_covariance[0] if len(msg.linear_acceleration_covariance) >= 1 else 1e9
        yaw_valid = yaw_cov < 1e5
        accel_valid = accel_cov < 1e5
        yaw_deg = None
        if yaw_valid:
            yaw_deg = wrap_angle_deg(yaw_deg_from_quat(msg.orientation.z, msg.orientation.w))

        state = ImuState(
            yaw_deg=yaw_deg,
            gz_deg_s=math.degrees(float(msg.angular_velocity.z)),
            ax=float(msg.linear_acceleration.x),
            ay=float(msg.linear_acceleration.y),
            az=float(msg.linear_acceleration.z),
            yaw_valid=yaw_valid,
            accel_valid=accel_valid,
        )
        self.latest_imu = state
        if self.start_imu_yaw_deg is None and yaw_deg is not None:
            self.start_imu_yaw_deg = yaw_deg

    @staticmethod
    def distance(a: OdomState, b: OdomState) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    @staticmethod
    def yaw_delta_deg(a_deg: float, b_deg: float) -> float:
        return wrap_angle_deg(a_deg - b_deg)

    def compute_static_active(self) -> bool:
        if self.latest_odom is None:
            return False
        odom_ok = (
            abs(self.latest_odom.vx) <= self.static_linear_threshold
            and abs(self.latest_odom.wz_deg_s) <= self.static_angular_threshold
        )
        if not odom_ok:
            return False
        if self.latest_imu is None:
            return True
        return abs(self.latest_imu.gz_deg_s) <= self.static_imu_gz_threshold

    def update_static_segment(self) -> None:
        active_now = self.compute_static_active()
        now_ns = self.get_clock().now().nanoseconds

        if active_now and not self.static_active:
            self.static_active = True
            self.static_started_ns = now_ns
            self.static_odom_start = self.latest_odom
            self.static_filtered_start = self.latest_filtered
            self.static_imu_start_yaw_deg = self.latest_imu.yaw_deg if self.latest_imu is not None else None
            return

        if not active_now and self.static_active:
            self.static_active = False
            self.static_started_ns = 0
            self.static_odom_start = None
            self.static_filtered_start = None
            self.static_imu_start_yaw_deg = None

    def on_timer(self) -> None:
        self.update_static_segment()
        now_s = self.get_clock().now().nanoseconds * 1e-9

        odom_vs_imu = None
        filtered_vs_imu = None
        filtered_vs_odom = None
        if self.latest_odom is not None and self.latest_imu is not None and self.latest_imu.yaw_deg is not None:
            odom_vs_imu = self.yaw_delta_deg(self.latest_odom.yaw_deg, self.latest_imu.yaw_deg)
        if self.latest_filtered is not None and self.latest_imu is not None and self.latest_imu.yaw_deg is not None:
            filtered_vs_imu = self.yaw_delta_deg(self.latest_filtered.yaw_deg, self.latest_imu.yaw_deg)
        if self.latest_filtered is not None and self.latest_odom is not None:
            filtered_vs_odom = self.yaw_delta_deg(self.latest_filtered.yaw_deg, self.latest_odom.yaw_deg)

        static_duration_s = 0.0
        static_odom_drift_m = None
        static_odom_yaw_drift_deg = None
        static_filtered_drift_m = None
        static_filtered_yaw_drift_deg = None
        static_imu_yaw_drift_deg = None

        if self.static_active:
            static_duration_s = (self.get_clock().now().nanoseconds - self.static_started_ns) * 1e-9
            if self.static_odom_start is not None and self.latest_odom is not None:
                static_odom_drift_m = self.distance(self.static_odom_start, self.latest_odom)
                static_odom_yaw_drift_deg = self.yaw_delta_deg(self.latest_odom.yaw_deg, self.static_odom_start.yaw_deg)
            if self.static_filtered_start is not None and self.latest_filtered is not None:
                static_filtered_drift_m = self.distance(self.static_filtered_start, self.latest_filtered)
                static_filtered_yaw_drift_deg = self.yaw_delta_deg(self.latest_filtered.yaw_deg, self.static_filtered_start.yaw_deg)
            if self.static_imu_start_yaw_deg is not None and self.latest_imu is not None and self.latest_imu.yaw_deg is not None:
                static_imu_yaw_drift_deg = self.yaw_delta_deg(self.latest_imu.yaw_deg, self.static_imu_start_yaw_deg)

        odom_line = 'odom: none'
        if self.latest_odom is not None:
            odom_line = (
                'odom: '
                f'x={self.latest_odom.x:.3f} y={self.latest_odom.y:.3f} '
                f'yaw={self.latest_odom.yaw_deg:.2f}deg '
                f'vx={self.latest_odom.vx:.3f}m/s wz={self.latest_odom.wz_deg_s:.2f}deg/s'
            )

        imu_line = 'imu: none'
        if self.latest_imu is not None:
            imu_line = (
                'imu: '
                f'yaw={finite_or_none(self.latest_imu.yaw_deg)}deg '
                f'gz={self.latest_imu.gz_deg_s:.2f}deg/s '
                f'acc=({self.latest_imu.ax:.3f},{self.latest_imu.ay:.3f},{self.latest_imu.az:.3f})'
            )

        filtered_line = 'filtered: none'
        if self.latest_filtered is not None:
            filtered_line = (
                'filtered: '
                f'x={self.latest_filtered.x:.3f} y={self.latest_filtered.y:.3f} '
                f'yaw={self.latest_filtered.yaw_deg:.2f}deg '
                f'vx={self.latest_filtered.vx:.3f}m/s wz={self.latest_filtered.wz_deg_s:.2f}deg/s'
            )

        diff_line = (
            'diff: '
            f'odom-imu={finite_or_none(odom_vs_imu)}deg '
            f'filtered-imu={finite_or_none(filtered_vs_imu)}deg '
            f'filtered-odom={finite_or_none(filtered_vs_odom)}deg'
        )

        static_line = (
            'static: '
            f'active={self.static_active} '
            f'dt={static_duration_s:.1f}s '
            f'odom_drift={finite_or_none(static_odom_drift_m)}m '
            f'odom_yaw_drift={finite_or_none(static_odom_yaw_drift_deg)}deg '
            f'filtered_drift={finite_or_none(static_filtered_drift_m)}m '
            f'filtered_yaw_drift={finite_or_none(static_filtered_yaw_drift_deg)}deg '
            f'imu_yaw_drift={finite_or_none(static_imu_yaw_drift_deg)}deg'
        )

        self.get_logger().info(' | '.join([odom_line, imu_line, filtered_line, diff_line, static_line]))

        if self.csv_writer is not None:
            self.csv_writer.writerow([
                f'{now_s:.3f}',
                self.latest_odom.x if self.latest_odom is not None else '',
                self.latest_odom.y if self.latest_odom is not None else '',
                self.latest_odom.yaw_deg if self.latest_odom is not None else '',
                self.latest_odom.vx if self.latest_odom is not None else '',
                self.latest_odom.wz_deg_s if self.latest_odom is not None else '',
                self.latest_imu.yaw_deg if self.latest_imu is not None and self.latest_imu.yaw_deg is not None else '',
                self.latest_imu.gz_deg_s if self.latest_imu is not None else '',
                self.latest_imu.ax if self.latest_imu is not None else '',
                self.latest_imu.ay if self.latest_imu is not None else '',
                self.latest_imu.az if self.latest_imu is not None else '',
                self.latest_filtered.x if self.latest_filtered is not None else '',
                self.latest_filtered.y if self.latest_filtered is not None else '',
                self.latest_filtered.yaw_deg if self.latest_filtered is not None else '',
                self.latest_filtered.vx if self.latest_filtered is not None else '',
                self.latest_filtered.wz_deg_s if self.latest_filtered is not None else '',
                odom_vs_imu if odom_vs_imu is not None else '',
                filtered_vs_imu if filtered_vs_imu is not None else '',
                filtered_vs_odom if filtered_vs_odom is not None else '',
                int(self.static_active),
                f'{static_duration_s:.3f}',
                static_odom_drift_m if static_odom_drift_m is not None else '',
                static_odom_yaw_drift_deg if static_odom_yaw_drift_deg is not None else '',
                static_filtered_drift_m if static_filtered_drift_m is not None else '',
                static_filtered_yaw_drift_deg if static_filtered_yaw_drift_deg is not None else '',
                static_imu_yaw_drift_deg if static_imu_yaw_drift_deg is not None else '',
            ])
            self.csv_file.flush()

    def close(self) -> None:
        if self.csv_file is not None:
            self.csv_file.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Live quantitative monitor for mapping odom/imu streams.')
    parser.add_argument('--print-hz', type=float, default=2.0, help='Console summary frequency.')
    parser.add_argument('--static-linear-threshold', type=float, default=0.02, help='Static vx threshold in m/s.')
    parser.add_argument('--static-angular-threshold', type=float, default=3.0, help='Static wz threshold in deg/s.')
    parser.add_argument('--static-imu-gz-threshold', type=float, default=3.0, help='Static imu gz threshold in deg/s.')
    parser.add_argument('--csv-out', default='', help='Optional CSV file path for continuous metric logging.')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = MappingMetricsMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
