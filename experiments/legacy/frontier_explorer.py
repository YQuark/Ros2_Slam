#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover - older rclpy fallback
    ExternalShutdownException = KeyboardInterrupt


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    half = yaw * 0.5
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class FrontierTarget:
    x: float
    y: float
    distance: float
    heading_error: float
    unknown_neighbors: int
    score: float


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('nav_lifecycle_node', '/bt_navigator')
        self.declare_parameter('tick_hz', 2.0)
        self.declare_parameter('status_log_interval_sec', 5.0)
        self.declare_parameter('startup_delay_sec', 4.0)
        self.declare_parameter('max_duration_sec', 180.0)
        self.declare_parameter('map_timeout_sec', 3.0)
        self.declare_parameter('tf_timeout_sec', 0.10)
        self.declare_parameter('goal_timeout_sec', 25.0)
        self.declare_parameter('frontier_min_distance', 0.45)
        self.declare_parameter('frontier_max_distance', 3.0)
        self.declare_parameter('frontier_sample_stride', 2)
        self.declare_parameter('min_frontier_cluster_cells', 6)
        self.declare_parameter('goal_approach_offset', 0.55)
        self.declare_parameter('goal_approach_max_offset', 0.90)
        self.declare_parameter('goal_approach_step', 0.05)
        self.declare_parameter('goal_clearance_radius', 0.38)
        self.declare_parameter('goal_unknown_clearance_radius', 0.35)
        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('min_unknown_neighbors', 1)
        self.declare_parameter('distance_weight', 0.45)
        self.declare_parameter('heading_weight', 0.75)
        self.declare_parameter('unknown_gain', 0.06)
        self.declare_parameter('blacklist_radius', 0.35)
        self.declare_parameter('blacklist_ttl_sec', 30.0)

        self.map_topic = str(self.get_parameter('map_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.navigate_action = str(self.get_parameter('navigate_action').value)
        self.nav_lifecycle_node = str(self.get_parameter('nav_lifecycle_node').value)
        self.tick_hz = max(0.5, float(self.get_parameter('tick_hz').value))
        self.status_log_interval_sec = max(0.0, float(self.get_parameter('status_log_interval_sec').value))
        self.startup_delay_sec = max(0.0, float(self.get_parameter('startup_delay_sec').value))
        self.max_duration_sec = max(0.0, float(self.get_parameter('max_duration_sec').value))
        self.map_timeout_sec = max(0.5, float(self.get_parameter('map_timeout_sec').value))
        self.tf_timeout_sec = max(0.0, float(self.get_parameter('tf_timeout_sec').value))
        self.goal_timeout_sec = max(1.0, float(self.get_parameter('goal_timeout_sec').value))
        self.frontier_min_distance = max(0.0, float(self.get_parameter('frontier_min_distance').value))
        self.frontier_max_distance = max(
            self.frontier_min_distance + 0.1,
            float(self.get_parameter('frontier_max_distance').value),
        )
        self.frontier_sample_stride = max(1, int(self.get_parameter('frontier_sample_stride').value))
        self.min_frontier_cluster_cells = max(1, int(self.get_parameter('min_frontier_cluster_cells').value))
        self.goal_approach_offset = max(0.0, float(self.get_parameter('goal_approach_offset').value))
        self.goal_approach_max_offset = max(
            self.goal_approach_offset,
            float(self.get_parameter('goal_approach_max_offset').value),
        )
        self.goal_approach_step = max(0.01, float(self.get_parameter('goal_approach_step').value))
        self.goal_clearance_radius = max(0.0, float(self.get_parameter('goal_clearance_radius').value))
        self.goal_unknown_clearance_radius = max(
            0.0,
            float(self.get_parameter('goal_unknown_clearance_radius').value),
        )
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.min_unknown_neighbors = max(1, int(self.get_parameter('min_unknown_neighbors').value))
        self.distance_weight = max(0.0, float(self.get_parameter('distance_weight').value))
        self.heading_weight = max(0.0, float(self.get_parameter('heading_weight').value))
        self.unknown_gain = max(0.0, float(self.get_parameter('unknown_gain').value))
        self.blacklist_radius = max(0.0, float(self.get_parameter('blacklist_radius').value))
        self.blacklist_ttl_sec = max(1.0, float(self.get_parameter('blacklist_ttl_sec').value))

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action)
        lifecycle_name = '/' + self.nav_lifecycle_node.strip('/')
        self.nav_state_client = self.create_client(GetState, f'{lifecycle_name}/get_state')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_msg: Optional[OccupancyGrid] = None
        self.last_map_time = None
        self.start_time = self.get_clock().now()
        self.active_goal_handle = None
        self.active_target: Optional[FrontierTarget] = None
        self.pending_target: Optional[FrontierTarget] = None
        self.active_goal_time = None
        self.blacklist: List[Tuple[float, float, float]] = []
        self.last_state = 'startup'
        self.last_stats: Dict[str, int] = {}
        self.last_log_sec = -1.0
        self.duration_cancel_sent = False
        self.nav_state_id: Optional[int] = None
        self.nav_state_label = 'unknown'
        self.nav_state_query_pending = False
        self.last_nav_state_query_time = None

        self.create_timer(1.0 / self.tick_hz, self.on_timer)
        self.get_logger().warn(
            'frontier_explorer active: '
            f'map={self.map_topic} action={self.navigate_action} frames={self.map_frame}->{self.base_frame}'
        )

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.last_map_time = self.get_clock().now()

    def elapsed_sec(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def map_age_sec(self) -> Optional[float]:
        if self.last_map_time is None:
            return None
        return (self.get_clock().now() - self.last_map_time).nanoseconds * 1e-9

    def map_is_fresh(self) -> bool:
        age = self.map_age_sec()
        return age is not None and 0.0 <= age <= self.map_timeout_sec

    def robot_pose(self) -> Optional[Tuple[float, float, float]]:
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
        return float(translation.x), float(translation.y), yaw_from_quaternion(rotation)

    def map_origin_yaw(self) -> float:
        return yaw_from_quaternion(self.map_msg.info.origin.orientation)

    def map_cell_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        info = self.map_msg.info
        origin = info.origin.position
        resolution = float(info.resolution)
        local_x = (float(grid_x) + 0.5) * resolution
        local_y = (float(grid_y) + 0.5) * resolution
        yaw = self.map_origin_yaw()
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            float(origin.x) + cos_yaw * local_x - sin_yaw * local_y,
            float(origin.y) + sin_yaw * local_x + cos_yaw * local_y,
        )

    def map_world_to_cell(self, world_x: float, world_y: float) -> Optional[Tuple[int, int]]:
        info = self.map_msg.info
        origin = info.origin.position
        resolution = max(1e-6, float(info.resolution))
        dx = world_x - float(origin.x)
        dy = world_y - float(origin.y)
        yaw = self.map_origin_yaw()
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        cell_x = int(math.floor(local_x / resolution))
        cell_y = int(math.floor(local_y / resolution))
        if cell_x < 1 or cell_y < 1 or cell_x >= int(info.width) - 1 or cell_y >= int(info.height) - 1:
            return None
        return cell_x, cell_y

    def is_free(self, value: int) -> bool:
        return 0 <= value <= self.free_threshold

    def is_occupied(self, value: int) -> bool:
        return value >= self.occupied_threshold

    def is_unknown(self, value: int) -> bool:
        return value < 0

    def unknown_neighbors(self, x: int, y: int) -> int:
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

    def is_frontier_cell(self, x: int, y: int) -> bool:
        info = self.map_msg.info
        width = int(info.width)
        value = self.map_msg.data[y * width + x]
        return self.is_free(value) and self.unknown_neighbors(x, y) >= self.min_unknown_neighbors

    def has_value_nearby(self, x: int, y: int, radius_m: float, predicate) -> bool:
        if radius_m <= 0.0:
            return False
        info = self.map_msg.info
        width = int(info.width)
        height = int(info.height)
        resolution = max(1e-6, float(info.resolution))
        radius_cells = max(1, int(math.ceil(radius_m / resolution)))
        data = self.map_msg.data
        for ny in range(max(1, y - radius_cells), min(height - 1, y + radius_cells + 1)):
            for nx in range(max(1, x - radius_cells), min(width - 1, x + radius_cells + 1)):
                if math.hypot(float(nx - x), float(ny - y)) > radius_cells:
                    continue
                if predicate(data[ny * width + nx]):
                    return True
        return False

    def has_occupied_nearby(self, x: int, y: int) -> bool:
        return self.has_value_nearby(x, y, self.goal_clearance_radius, self.is_occupied)

    def has_unknown_nearby(self, x: int, y: int) -> bool:
        return self.has_value_nearby(x, y, self.goal_unknown_clearance_radius, self.is_unknown)

    def goal_safety_reject_reason(self, x: int, y: int) -> str:
        info = self.map_msg.info
        width = int(info.width)
        value = self.map_msg.data[y * width + x]
        if not self.is_free(value):
            return 'not_free'
        if self.has_occupied_nearby(x, y):
            return 'occupied_near'
        if self.has_unknown_nearby(x, y):
            return 'unknown_near'
        return ''

    def find_safe_approach_cell(
        self,
        frontier_x: float,
        frontier_y: float,
        robot_x: float,
        robot_y: float,
    ) -> Tuple[Optional[int], Optional[int], float, float, str]:
        dx = frontier_x - robot_x
        dy = frontier_y - robot_y
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            return None, None, frontier_x, frontier_y, 'too_close'

        unit_x = dx / distance
        unit_y = dy / distance
        max_offset = min(self.goal_approach_max_offset, max(0.0, distance - 0.05))
        offset = min(self.goal_approach_offset, max_offset)
        last_reason = 'no_room'

        while offset <= max_offset + 1e-9:
            target_x = frontier_x - unit_x * offset
            target_y = frontier_y - unit_y * offset
            cell = self.map_world_to_cell(target_x, target_y)
            if cell is None:
                last_reason = 'off_map'
            else:
                reason = self.goal_safety_reject_reason(cell[0], cell[1])
                if not reason:
                    return cell[0], cell[1], target_x, target_y, ''
                last_reason = reason
            offset += self.goal_approach_step

        return None, None, frontier_x, frontier_y, last_reason

    def prune_blacklist(self) -> None:
        now = self.elapsed_sec()
        self.blacklist = [(x, y, expiry) for x, y, expiry in self.blacklist if expiry > now]

    def is_blacklisted(self, x: float, y: float) -> bool:
        self.prune_blacklist()
        for bx, by, _ in self.blacklist:
            if math.hypot(x - bx, y - by) <= self.blacklist_radius:
                return True
        return False

    def blacklist_target(self, target: Optional[FrontierTarget], reason: str) -> None:
        if target is None:
            return
        expiry = self.elapsed_sec() + self.blacklist_ttl_sec
        self.blacklist.append((target.x, target.y, expiry))
        self.get_logger().warn(
            f'frontier_blacklist reason={reason} target=({target.x:.2f},{target.y:.2f}) '
            f'ttl={self.blacklist_ttl_sec:.1f}s'
        )

    def find_frontier_target(self, pose: Tuple[float, float, float]) -> Optional[FrontierTarget]:
        robot_x, robot_y, robot_yaw = pose
        info = self.map_msg.info
        width = int(info.width)
        height = int(info.height)
        data = self.map_msg.data
        stats = {
            'free': 0,
            'frontier': 0,
            'clusters': 0,
            'small_clusters': 0,
            'candidates': 0,
            'too_close': 0,
            'too_far': 0,
            'blocked': 0,
            'approach_blocked': 0,
            'approach_unknown': 0,
            'approach_no_room': 0,
            'approach_not_free': 0,
            'approach_unknown_near': 0,
            'approach_off_map': 0,
            'approach_too_close': 0,
            'blacklisted': 0,
        }
        best = None
        best_score = math.inf

        if width < 3 or height < 3 or len(data) < width * height:
            self.last_stats = stats
            return None

        frontier = bytearray(width * height)
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                if self.is_free(data[row + x]):
                    stats['free'] += 1
                if self.is_frontier_cell(x, y):
                    frontier[row + x] = 1
                    stats['frontier'] += 1

        visited = bytearray(width * height)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                start_idx = y * width + x
                if not frontier[start_idx] or visited[start_idx]:
                    continue

                cluster: List[Tuple[int, int]] = []
                stack = [(x, y)]
                visited[start_idx] = 1
                while stack:
                    cx, cy = stack.pop()
                    cluster.append((cx, cy))
                    for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                        if nx <= 0 or ny <= 0 or nx >= width - 1 or ny >= height - 1:
                            continue
                        idx = ny * width + nx
                        if not frontier[idx] or visited[idx]:
                            continue
                        visited[idx] = 1
                        stack.append((nx, ny))

                stats['clusters'] += 1
                if len(cluster) < self.min_frontier_cluster_cells:
                    stats['small_clusters'] += 1
                    continue

                centroid_x = sum(cell[0] for cell in cluster) / float(len(cluster))
                centroid_y = sum(cell[1] for cell in cluster) / float(len(cluster))
                cluster_bonus = self.unknown_gain * math.sqrt(float(len(cluster)))
                ordered_cluster = sorted(
                    cluster[::self.frontier_sample_stride],
                    key=lambda cell: math.hypot(cell[0] - centroid_x, cell[1] - centroid_y),
                )

                for x, y in ordered_cluster:
                    stats['candidates'] += 1

                    frontier_x, frontier_y = self.map_cell_to_world(x, y)
                    _, _, target_x, target_y, reject_reason = self.find_safe_approach_cell(
                        frontier_x,
                        frontier_y,
                        robot_x,
                        robot_y,
                    )
                    if reject_reason:
                        if reject_reason == 'occupied_near':
                            stats['blocked'] += 1
                            stats['approach_blocked'] += 1
                        elif reject_reason == 'not_free':
                            stats['approach_unknown'] += 1
                            stats['approach_not_free'] += 1
                        elif reject_reason == 'unknown_near':
                            stats['approach_unknown'] += 1
                            stats['approach_unknown_near'] += 1
                        elif reject_reason == 'off_map':
                            stats['approach_unknown'] += 1
                            stats['approach_off_map'] += 1
                        elif reject_reason == 'too_close':
                            stats['approach_too_close'] += 1
                        else:
                            stats['approach_no_room'] += 1
                        continue

                    if self.is_blacklisted(target_x, target_y):
                        stats['blacklisted'] += 1
                        continue

                    dx = target_x - robot_x
                    dy = target_y - robot_y
                    distance = math.hypot(dx, dy)
                    if distance < self.frontier_min_distance:
                        stats['too_close'] += 1
                        continue
                    if distance > self.frontier_max_distance:
                        stats['too_far'] += 1
                        continue

                    heading_error = normalize_angle(math.atan2(dy, dx) - robot_yaw)
                    score = (
                        self.distance_weight * distance
                        + self.heading_weight * abs(heading_error)
                        - cluster_bonus
                    )
                    if score < best_score:
                        best_score = score
                        best = FrontierTarget(
                            x=target_x,
                            y=target_y,
                            distance=distance,
                            heading_error=heading_error,
                            unknown_neighbors=len(cluster),
                            score=score,
                        )

        self.last_stats = stats
        return best

    def make_goal(self, target: FrontierTarget, pose: Tuple[float, float, float]) -> NavigateToPose.Goal:
        robot_x, robot_y, _ = pose
        yaw = math.atan2(target.y - robot_y, target.x - robot_x)
        q = yaw_to_quaternion(yaw)

        stamped = PoseStamped()
        stamped.header.frame_id = self.map_frame
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose.position.x = target.x
        stamped.pose.position.y = target.y
        stamped.pose.position.z = 0.0
        stamped.pose.orientation.x = q.x
        stamped.pose.orientation.y = q.y
        stamped.pose.orientation.z = q.z
        stamped.pose.orientation.w = q.w

        goal = NavigateToPose.Goal()
        goal.pose = stamped
        return goal

    def send_goal(self, target: FrontierTarget, pose: Tuple[float, float, float]) -> None:
        goal = self.make_goal(target, pose)
        self.pending_target = target
        self.last_state = 'sending_goal'
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
        self.get_logger().info(
            f'frontier_goal_send target=({target.x:.2f},{target.y:.2f}) '
            f'distance={target.distance:.2f} heading={math.degrees(target.heading_error):.1f}deg '
            f'unknown={target.unknown_neighbors} score={target.score:.3f}'
        )

    def on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'frontier_goal_send_failed error={exc}')
            self.blacklist_target(self.pending_target, 'send_exception')
            self.pending_target = None
            self.active_goal_handle = None
            self.active_target = None
            self.active_goal_time = None
            self.last_state = 'send_failed'
            return

        if not goal_handle.accepted:
            self.get_logger().warn('frontier_goal_rejected')
            self.blacklist_target(self.pending_target, 'rejected')
            self.pending_target = None
            self.active_goal_handle = None
            self.active_target = None
            self.active_goal_time = None
            self.last_state = 'goal_rejected'
            return

        self.active_goal_handle = goal_handle
        self.active_target = self.pending_target
        self.pending_target = None
        self.active_goal_time = self.get_clock().now()
        self.last_state = 'navigating'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future) -> None:
        target = self.active_target
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self.get_logger().error(f'frontier_goal_result_failed error={exc}')
            self.blacklist_target(target, 'result_exception')
            status = 0

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('frontier_goal_succeeded')
            self.last_state = 'goal_succeeded'
        else:
            self.get_logger().warn(f'frontier_goal_failed status={status}')
            self.blacklist_target(target, f'status_{status}')
            self.last_state = f'goal_failed_{status}'

        self.active_goal_handle = None
        self.active_target = None
        self.active_goal_time = None

    def cancel_active_goal(self, reason: str) -> None:
        if self.active_goal_handle is None:
            return
        try:
            self.active_goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warn(f'frontier_cancel_failed reason={reason} error={exc}')
        self.get_logger().warn(f'frontier_goal_cancel reason={reason}')
        self.blacklist_target(self.active_target, reason)
        self.active_goal_handle = None
        self.active_target = None
        self.active_goal_time = None
        self.last_state = f'cancel_{reason}'

    def active_goal_age(self) -> float:
        if self.active_goal_time is None:
            return 0.0
        return (self.get_clock().now() - self.active_goal_time).nanoseconds * 1e-9

    def request_nav_state(self) -> None:
        now = self.get_clock().now()
        if self.last_nav_state_query_time is not None:
            age = (now - self.last_nav_state_query_time).nanoseconds * 1e-9
            if age < 1.0:
                return
        if self.nav_state_query_pending:
            return
        if not self.nav_state_client.wait_for_service(timeout_sec=0.0):
            self.nav_state_id = None
            self.nav_state_label = 'state_service_unavailable'
            self.last_nav_state_query_time = now
            return

        self.nav_state_query_pending = True
        self.last_nav_state_query_time = now
        future = self.nav_state_client.call_async(GetState.Request())
        future.add_done_callback(self.on_nav_state_response)

    def on_nav_state_response(self, future) -> None:
        self.nav_state_query_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.nav_state_id = None
            self.nav_state_label = f'state_error:{exc}'
            return
        self.nav_state_id = int(response.current_state.id)
        self.nav_state_label = str(response.current_state.label)

    def nav_server_ready(self) -> bool:
        self.request_nav_state()

        try:
            if self.nav_client.server_is_ready():
                action_ready = True
            else:
                action_ready = False
        except AttributeError:
            action_ready = self.nav_client.wait_for_server(timeout_sec=0.10)
        if not action_ready:
            self.last_state = 'waiting_nav2_action'
            return False
        if self.nav_state_id != State.PRIMARY_STATE_ACTIVE:
            self.last_state = f'waiting_nav2_{self.nav_state_label}'
            return False
        return True

    def on_timer(self) -> None:
        elapsed = self.elapsed_sec()
        if elapsed < self.startup_delay_sec:
            self.last_state = 'startup_delay'
            self.log_status()
            return

        if self.max_duration_sec > 0.0 and elapsed >= self.max_duration_sec:
            if not self.duration_cancel_sent:
                self.cancel_active_goal('duration_timeout')
                self.duration_cancel_sent = True
            self.last_state = 'duration_timeout'
            self.log_status()
            return

        if not self.nav_server_ready():
            self.log_status()
            return

        if self.map_msg is None or not self.map_is_fresh():
            self.last_state = 'waiting_map'
            self.log_status()
            return

        pose = self.robot_pose()
        if pose is None:
            self.last_state = 'waiting_tf'
            self.log_status()
            return

        if self.active_goal_handle is not None:
            if self.active_goal_age() > self.goal_timeout_sec:
                self.cancel_active_goal('goal_timeout')
            else:
                self.last_state = 'navigating'
            self.log_status()
            return

        target = self.find_frontier_target(pose)
        if target is None:
            self.last_state = 'no_frontier'
            self.log_status()
            return

        self.send_goal(target, pose)
        self.log_status()

    def log_status(self) -> None:
        if self.status_log_interval_sec <= 0.0:
            return
        elapsed = self.elapsed_sec()
        if self.last_log_sec >= 0.0 and (elapsed - self.last_log_sec) < self.status_log_interval_sec:
            return
        self.last_log_sec = elapsed

        map_age = self.map_age_sec()
        map_age_text = 'none' if map_age is None else f'{map_age:.2f}s'
        active = 'none'
        if self.active_target is not None:
            active = (
                f'({self.active_target.x:.2f},{self.active_target.y:.2f},'
                f'{self.active_target.distance:.2f}m,{math.degrees(self.active_target.heading_error):.1f}deg)'
            )
        stats = ','.join(f'{key}={value}' for key, value in sorted(self.last_stats.items()))
        self.get_logger().info(
            'explorer_state '
            f'state={self.last_state} elapsed={elapsed:.1f}s map_age={map_age_text} '
            f'nav={self.nav_state_label} active={active} blacklist={len(self.blacklist)} stats=[{stats}]'
        )

    def shutdown(self) -> None:
        self.cancel_active_goal('shutdown')
        try:
            self.nav_client.destroy()
        except Exception:
            pass
        try:
            self.nav_state_client.destroy()
        except Exception:
            pass


def main() -> int:
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.shutdown()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
