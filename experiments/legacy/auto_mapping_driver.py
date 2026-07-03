#!/usr/bin/env python3

import math
from typing import Iterable, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover - older rclpy fallback
    ExternalShutdownException = KeyboardInterrupt


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _normalize_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class AutoMappingDriver(Node):
    def __init__(self) -> None:
        super().__init__('auto_mapping_driver')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_frontier_exploration', True)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('startup_delay_sec', 5.0)
        self.declare_parameter('max_duration_sec', 180.0)
        self.declare_parameter('scan_timeout_sec', 1.0)
        self.declare_parameter('map_timeout_sec', 3.0)
        self.declare_parameter('tf_timeout_sec', 0.05)
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('turn_speed', 1.65)
        self.declare_parameter('emergency_stop_distance', 0.25)
        self.declare_parameter('front_stop_distance', 0.40)
        self.declare_parameter('front_resume_distance', 0.48)
        self.declare_parameter('front_slow_distance', 0.80)
        self.declare_parameter('side_emergency_stop_distance', 0.12)
        self.declare_parameter('side_stop_distance', 0.10)
        self.declare_parameter('side_resume_distance', 0.18)
        self.declare_parameter('front_angle_deg', 35.0)
        self.declare_parameter('side_min_angle_deg', 35.0)
        self.declare_parameter('side_max_angle_deg', 110.0)
        self.declare_parameter('linear_accel_limit', 0.08)
        self.declare_parameter('angular_accel_limit', 0.50)
        self.declare_parameter('steer_gain', 0.35)
        self.declare_parameter('heading_gain', 0.90)
        self.declare_parameter('max_drive_angular_speed', 0.70)
        self.declare_parameter('frontier_min_distance', 0.45)
        self.declare_parameter('frontier_max_distance', 3.50)
        self.declare_parameter('frontier_goal_tolerance', 0.25)
        self.declare_parameter('frontier_sample_stride', 2)
        self.declare_parameter('frontier_rotate_angle_deg', 80.0)
        self.declare_parameter('frontier_heading_weight', 0.80)
        self.declare_parameter('frontier_distance_weight', 0.35)
        self.declare_parameter('frontier_unknown_gain', 0.05)
        self.declare_parameter('frontier_forward_only', True)
        self.declare_parameter('status_log_interval_sec', 1.0)
        self.declare_parameter('base_max_linear', 1.20)
        self.declare_parameter('base_max_angular', 19.27)
        self.declare_parameter('cmd_deadzone_norm', 0.04)
        self.declare_parameter('cmd_deadzone_margin_norm', 0.005)
        self.declare_parameter('cmd_min_effective_norm', 0.085)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.use_frontier_exploration = _as_bool(self.get_parameter('use_frontier_exploration').value)
        self.publish_hz = max(1.0, float(self.get_parameter('publish_hz').value))
        self.startup_delay_sec = max(0.0, float(self.get_parameter('startup_delay_sec').value))
        self.max_duration_sec = max(0.0, float(self.get_parameter('max_duration_sec').value))
        self.scan_timeout_sec = max(0.1, float(self.get_parameter('scan_timeout_sec').value))
        self.map_timeout_sec = max(0.5, float(self.get_parameter('map_timeout_sec').value))
        self.tf_timeout_sec = max(0.0, float(self.get_parameter('tf_timeout_sec').value))
        self.base_max_linear = max(1e-6, float(self.get_parameter('base_max_linear').value))
        self.base_max_angular = max(1e-6, float(self.get_parameter('base_max_angular').value))
        self.cmd_deadzone_norm = max(0.0, float(self.get_parameter('cmd_deadzone_norm').value))
        self.cmd_deadzone_margin_norm = max(0.0, float(self.get_parameter('cmd_deadzone_margin_norm').value))
        self.cmd_min_effective_norm = max(0.0, float(self.get_parameter('cmd_min_effective_norm').value))
        self.min_cmd_norm = max(
            self.cmd_deadzone_norm + self.cmd_deadzone_margin_norm,
            self.cmd_min_effective_norm,
        )
        self.min_executable_linear = self.base_max_linear * self.min_cmd_norm
        self.min_executable_angular = self.base_max_angular * self.min_cmd_norm
        requested_linear_speed = max(0.0, float(self.get_parameter('linear_speed').value))
        requested_turn_speed = abs(float(self.get_parameter('turn_speed').value))
        self.linear_speed = self.make_executable_target(requested_linear_speed, self.min_executable_linear)
        self.turn_speed = self.make_executable_target(requested_turn_speed, self.min_executable_angular)
        self.warn_if_deadzone_adjusted('linear_speed', requested_linear_speed, self.linear_speed)
        self.warn_if_deadzone_adjusted('turn_speed', requested_turn_speed, self.turn_speed)
        self.emergency_stop_distance = max(0.05, float(self.get_parameter('emergency_stop_distance').value))
        self.front_stop_distance = max(0.05, float(self.get_parameter('front_stop_distance').value))
        self.front_resume_distance = max(
            self.front_stop_distance + 0.05,
            float(self.get_parameter('front_resume_distance').value),
        )
        self.front_slow_distance = max(
            self.front_resume_distance + 0.05,
            float(self.get_parameter('front_slow_distance').value),
        )
        self.side_emergency_stop_distance = max(
            0.05,
            float(self.get_parameter('side_emergency_stop_distance').value),
        )
        self.side_stop_distance = max(0.05, float(self.get_parameter('side_stop_distance').value))
        self.side_resume_distance = max(
            self.side_stop_distance + 0.03,
            float(self.get_parameter('side_resume_distance').value),
        )
        self.front_angle = math.radians(abs(float(self.get_parameter('front_angle_deg').value)))
        self.side_min_angle = math.radians(abs(float(self.get_parameter('side_min_angle_deg').value)))
        self.side_max_angle = math.radians(abs(float(self.get_parameter('side_max_angle_deg').value)))
        if self.side_max_angle <= self.side_min_angle:
            self.side_max_angle = self.side_min_angle + math.radians(20.0)
        self.linear_accel_limit = max(0.01, float(self.get_parameter('linear_accel_limit').value))
        self.angular_accel_limit = max(0.05, float(self.get_parameter('angular_accel_limit').value))
        self.steer_gain = max(0.0, float(self.get_parameter('steer_gain').value))
        self.heading_gain = max(0.0, float(self.get_parameter('heading_gain').value))
        self.max_drive_angular_speed = max(0.0, float(self.get_parameter('max_drive_angular_speed').value))
        self.frontier_min_distance = max(0.0, float(self.get_parameter('frontier_min_distance').value))
        self.frontier_max_distance = max(
            self.frontier_min_distance + 0.1,
            float(self.get_parameter('frontier_max_distance').value),
        )
        self.frontier_goal_tolerance = max(0.05, float(self.get_parameter('frontier_goal_tolerance').value))
        self.frontier_sample_stride = max(1, int(self.get_parameter('frontier_sample_stride').value))
        self.frontier_rotate_angle = math.radians(abs(float(self.get_parameter('frontier_rotate_angle_deg').value)))
        self.frontier_heading_weight = max(0.0, float(self.get_parameter('frontier_heading_weight').value))
        self.frontier_distance_weight = max(0.0, float(self.get_parameter('frontier_distance_weight').value))
        self.frontier_unknown_gain = max(0.0, float(self.get_parameter('frontier_unknown_gain').value))
        self.frontier_forward_only = _as_bool(self.get_parameter('frontier_forward_only').value)
        self.status_log_interval_sec = max(0.0, float(self.get_parameter('status_log_interval_sec').value))

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.on_scan,
            qos_profile_sensor_data,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.last_scan_time = None
        self.map_msg = None
        self.last_map_time = None
        self.start_time = self.get_clock().now()
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.stopped_by_timeout = False
        self.last_reason = 'startup'
        self.last_log_sec = -1.0
        self.hazard_latched = False
        self.turn_direction = 1.0
        self.frontier_target = None

        self.create_timer(1.0 / self.publish_hz, self.on_timer)
        self.get_logger().warn(
            f'auto_mapping_driver active: scan={self.scan_topic} map={self.map_topic} '
            f'cmd_vel={self.cmd_vel_topic} frontier={self.use_frontier_exploration}; '
            f'using sensor_data QoS; min_executable=({self.min_executable_linear:.3f}m/s,'
            f'{self.min_executable_angular:.3f}rad/s); keep a physical emergency stop ready.'
        )

    def make_executable_target(self, value: float, minimum: float) -> float:
        if value <= 0.0:
            return 0.0
        return max(value, minimum)

    def warn_if_deadzone_adjusted(self, name: str, requested: float, effective: float) -> None:
        if requested > 0.0 and effective > requested + 1e-6:
            self.get_logger().warn(
                f'{name} raised from {requested:.3f} to {effective:.3f} to exceed STM32 normalized deadzone'
            )

    def on_scan(self, msg: LaserScan) -> None:
        self.front_min = self.min_range_in_sector(msg, -self.front_angle, self.front_angle)
        self.left_min = self.min_range_in_sector(msg, self.side_min_angle, self.side_max_angle)
        self.right_min = self.min_range_in_sector(msg, -self.side_max_angle, -self.side_min_angle)
        self.last_scan_time = self.get_clock().now()

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.last_map_time = self.get_clock().now()

    def iter_sector_ranges(self, msg: LaserScan, start_angle: float, end_angle: float) -> Iterable[float]:
        lo = min(start_angle, end_angle)
        hi = max(start_angle, end_angle)
        angle = float(msg.angle_min)
        inc = float(msg.angle_increment)
        if abs(inc) < 1e-9:
            return
        for value in msg.ranges:
            if lo <= angle <= hi and math.isfinite(value):
                if msg.range_min <= value <= msg.range_max:
                    yield float(value)
            angle += inc

    def min_range_in_sector(self, msg: LaserScan, start_angle: float, end_angle: float) -> float:
        values = list(self.iter_sector_ranges(msg, start_angle, end_angle))
        if not values:
            return math.inf
        return min(values)

    def scan_is_fresh(self) -> bool:
        if self.last_scan_time is None:
            return False
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
        return 0.0 <= age <= self.scan_timeout_sec

    def scan_age_sec(self) -> Optional[float]:
        if self.last_scan_time is None:
            return None
        return (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9

    def map_is_fresh(self) -> bool:
        if self.last_map_time is None:
            return False
        age = (self.get_clock().now() - self.last_map_time).nanoseconds * 1e-9
        return 0.0 <= age <= self.map_timeout_sec

    def elapsed_sec(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def choose_turn_direction(self) -> float:
        if self.left_min == math.inf and self.right_min == math.inf:
            return self.turn_direction
        if self.left_min >= self.right_min:
            return 1.0
        return -1.0

    def side_min_clearance(self) -> float:
        return min(self.left_min, self.right_min)

    def update_turn_latch(self) -> None:
        self.turn_direction = self.choose_turn_direction()

    def hazard_clear(self) -> bool:
        return self.scan_is_fresh() and self.front_min >= self.front_resume_distance

    def robot_pose_in_map(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        translation = tf.transform.translation
        rotation = tf.transform.rotation
        return float(translation.x), float(translation.y), _yaw_from_quaternion(rotation)

    def map_cell_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        info = self.map_msg.info
        origin = info.origin.position
        resolution = float(info.resolution)
        return (
            float(origin.x) + (float(grid_x) + 0.5) * resolution,
            float(origin.y) + (float(grid_y) + 0.5) * resolution,
        )

    def frontier_unknown_neighbors(self, x: int, y: int) -> int:
        info = self.map_msg.info
        width = int(info.width)
        data = self.map_msg.data
        count = 0
        for ny in range(y - 1, y + 2):
            for nx in range(x - 1, x + 2):
                if nx == x and ny == y:
                    continue
                if data[ny * width + nx] < 0:
                    count += 1
        return count

    def find_frontier_target(self) -> Optional[Tuple[float, float, float, float]]:
        if not self.use_frontier_exploration or self.map_msg is None or not self.map_is_fresh():
            self.frontier_target = None
            return None

        pose = self.robot_pose_in_map()
        if pose is None:
            self.frontier_target = None
            return None

        robot_x, robot_y, robot_yaw = pose
        info = self.map_msg.info
        width = int(info.width)
        height = int(info.height)
        data = self.map_msg.data
        if width < 3 or height < 3 or len(data) < width * height:
            self.frontier_target = None
            return None

        best = None
        best_score = math.inf
        stride = self.frontier_sample_stride
        for y in range(1, height - 1, stride):
            row = y * width
            for x in range(1, width - 1, stride):
                value = data[row + x]
                if value < 0 or value > 20:
                    continue
                unknown_neighbors = self.frontier_unknown_neighbors(x, y)
                if unknown_neighbors <= 0:
                    continue

                target_x, target_y = self.map_cell_to_world(x, y)
                dx = target_x - robot_x
                dy = target_y - robot_y
                distance = math.hypot(dx, dy)
                if distance < self.frontier_min_distance or distance > self.frontier_max_distance:
                    continue

                heading_error = _normalize_angle(math.atan2(dy, dx) - robot_yaw)
                if self.frontier_forward_only and abs(heading_error) > (math.pi * 0.5):
                    continue
                score = (
                    self.frontier_distance_weight * distance
                    + self.frontier_heading_weight * abs(heading_error)
                    - self.frontier_unknown_gain * float(min(unknown_neighbors, 8))
                )
                if score < best_score:
                    best_score = score
                    best = (target_x, target_y, distance, heading_error)

        self.frontier_target = best
        return best

    def frontier_drive_target(self) -> Optional[Tuple[float, float, str]]:
        frontier = self.find_frontier_target()
        if frontier is None:
            return None

        _, _, distance, heading_error = frontier
        if distance <= self.frontier_goal_tolerance:
            return None

        turn = _clamp(
            self.heading_gain * heading_error,
            -self.max_drive_angular_speed,
            self.max_drive_angular_speed,
        )
        speed = self.linear_speed
        if self.front_min < self.front_slow_distance:
            span = self.front_slow_distance - self.front_stop_distance
            ratio = _clamp((self.front_min - self.front_stop_distance) / span, 0.35, 1.0)
            speed *= ratio
        if abs(heading_error) > self.frontier_rotate_angle:
            speed *= 0.5
            return speed, turn, 'frontier_align_drive'
        return speed, turn, 'frontier_drive'

    def compute_target(self) -> Tuple[float, float, str]:
        elapsed = self.elapsed_sec()
        if elapsed < self.startup_delay_sec:
            self.hazard_latched = False
            return 0.0, 0.0, 'startup_delay'
        if self.max_duration_sec > 0.0 and elapsed >= self.max_duration_sec:
            if not self.stopped_by_timeout:
                self.get_logger().warn('auto mapping duration reached; holding zero velocity')
                self.stopped_by_timeout = True
            self.hazard_latched = True
            return 0.0, 0.0, 'duration_timeout'
        if not self.scan_is_fresh():
            self.hazard_latched = True
            return 0.0, 0.0, 'stale_scan'

        if self.front_min <= self.emergency_stop_distance:
            self.hazard_latched = True
            self.update_turn_latch()
            return 0.0, 0.0, 'emergency_stop'

        if self.front_min <= self.front_stop_distance:
            self.hazard_latched = True
            self.update_turn_latch()
            return 0.0, self.turn_direction * self.turn_speed, 'front_blocked'

        if self.hazard_latched and not self.hazard_clear():
            return 0.0, self.turn_direction * self.turn_speed, 'clearing_hazard'

        self.hazard_latched = False

        frontier_target = self.frontier_drive_target()
        if frontier_target is not None:
            return frontier_target

        side_error = 0.0
        if math.isfinite(self.left_min) and math.isfinite(self.right_min):
            side_error = self.left_min - self.right_min
        steer = _clamp(self.steer_gain * side_error, -self.turn_speed, self.turn_speed)

        if self.front_min < self.front_slow_distance:
            span = self.front_slow_distance - self.front_stop_distance
            ratio = _clamp((self.front_min - self.front_stop_distance) / span, 0.25, 1.0)
            return self.linear_speed * ratio, steer, 'front_slow'

        return self.linear_speed, steer, 'forward'

    def ramp(self, current: float, target: float, limit_per_sec: float, dt: float) -> float:
        step = limit_per_sec * dt
        return current + _clamp(target - current, -step, step)

    def enforce_executable_step(self, current: float, target: float, minimum: float) -> float:
        if abs(target) <= 1e-6 or abs(current) <= 1e-6:
            return current
        if abs(current) < minimum:
            return math.copysign(minimum, target)
        return current

    def on_timer(self) -> None:
        target_vx, target_wz, reason = self.compute_target()
        dt = 1.0 / self.publish_hz
        if reason in ('emergency_stop', 'stale_scan', 'duration_timeout', 'startup_delay'):
            self.current_vx = 0.0
            self.current_wz = 0.0
        else:
            if target_vx <= 0.0 and reason in ('front_blocked', 'clearing_hazard', 'frontier_turn'):
                self.current_vx = 0.0
            else:
                self.current_vx = self.ramp(self.current_vx, target_vx, self.linear_accel_limit, dt)
                self.current_vx = self.enforce_executable_step(
                    self.current_vx,
                    target_vx,
                    self.min_executable_linear,
                )
            self.current_wz = self.ramp(self.current_wz, target_wz, self.angular_accel_limit, dt)
            if reason in ('front_blocked', 'clearing_hazard', 'frontier_turn'):
                self.current_wz = self.enforce_executable_step(
                    self.current_wz,
                    target_wz,
                    self.min_executable_angular,
                )
        self.last_reason = reason
        self.log_status(target_vx, target_wz, reason)
        self.publish_cmd(self.current_vx, self.current_wz)

    def log_status(self, target_vx: float, target_wz: float, reason: str) -> None:
        if self.status_log_interval_sec <= 0.0:
            return
        elapsed = self.elapsed_sec()
        if self.last_log_sec >= 0.0 and (elapsed - self.last_log_sec) < self.status_log_interval_sec:
            return
        self.last_log_sec = elapsed
        scan_age = self.scan_age_sec()
        scan_age_text = 'none' if scan_age is None else f'{scan_age:.2f}s'
        frontier_text = 'none'
        if self.frontier_target is not None:
            fx, fy, fd, fe = self.frontier_target
            frontier_text = f'({fx:.2f},{fy:.2f},{fd:.2f}m,{math.degrees(fe):.1f}deg)'
        self.get_logger().info(
            'auto_mapping_state '
            f'reason={reason} elapsed={elapsed:.1f}s scan_age={scan_age_text} '
            f'front={self.front_min:.2f} left={self.left_min:.2f} right={self.right_min:.2f} '
            f'frontier={frontier_text} '
            f'target=({target_vx:.3f},{target_wz:.3f}) '
            f'cmd=({self.current_vx:.3f},{self.current_wz:.3f})'
        )

    def publish_cmd(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def stop(self) -> None:
        self.current_vx = 0.0
        self.current_wz = 0.0
        for _ in range(3):
            try:
                self.publish_cmd(0.0, 0.0)
            except Exception:
                break


def main() -> int:
    rclpy.init()
    node = AutoMappingDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
