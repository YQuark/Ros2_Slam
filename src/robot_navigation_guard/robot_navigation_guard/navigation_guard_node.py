#!/usr/bin/env python3
"""Fail-closed public NavigateToPose proxy with cancel-and-reissue semantics."""

import asyncio
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from robot_interfaces.msg import (
    ChassisLinkState,
    FirmwareControlState,
    HostControlState,
    MotionSupervisionState,
    NavigationGuardState,
    PlatformCompatibilityState,
)

from robot_navigation_guard.guard import GoalGuard


class NavigationGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_guard")
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.declare_parameter("public_action", "navigate_to_pose")
        self.declare_parameter("backend_action", "nav2/navigate_to_pose")
        self.declare_parameter("cancel_retry_sec", 0.20)
        self.guard = GoalGuard()
        self.link_safe = self.firmware_safe = self.supervision_safe = False
        self.compatible = self.host_safe = False
        self.reason_flags = (
            NavigationGuardState.REASON_LINK
            | NavigationGuardState.REASON_FIRMWARE
            | NavigationGuardState.REASON_SUPERVISION
            | NavigationGuardState.REASON_COMPATIBILITY
            | NavigationGuardState.REASON_HOST_CONTROL
        )
        self.backend_handle = None
        self.last_cancel_at = None
        self.cancel_retry_sec = float(self.get_parameter("cancel_retry_sec").value)
        self.backend = ActionClient(
            self, NavigateToPose, str(self.get_parameter("backend_action").value)
        )
        self.server = ActionServer(
            self,
            NavigateToPose,
            str(self.get_parameter("public_action").value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self.publisher = self.create_publisher(NavigationGuardState, "navigation/guard_state", 10)
        self.create_subscription(ChassisLinkState, "chassis/link_state", self._on_link, 10)
        self.create_subscription(
            FirmwareControlState, "chassis/firmware_control_state", self._on_firmware, 10
        )
        self.create_subscription(
            MotionSupervisionState, "motion/supervision_state", self._on_supervision, 10
        )
        self.create_subscription(
            PlatformCompatibilityState, "platform/compatibility_state", self._on_compatibility, 10
        )
        self.create_subscription(HostControlState, "chassis/host_control_state", self._on_host, 10)
        self.create_timer(0.05, self._cancel_tick, clock=self.steady_clock)

    def _on_link(self, msg) -> None:
        self.link_safe = bool(
            msg.link_state
            in (ChassisLinkState.STATE_WIRE_REARM_READY, ChassisLinkState.STATE_DRIVE_ACTIVE)
            and not msg.wire_rearm_required
            and msg.protocol_compatible
        )
        self._evaluate()

    def _on_firmware(self, msg) -> None:
        self.firmware_safe = not msg.estop_active and not msg.fault_stop_active
        self._evaluate()

    def _on_supervision(self, msg) -> None:
        self.supervision_safe = bool(
            msg.level != MotionSupervisionState.LEVEL_CRITICAL and not msg.release_host_candidate
        )
        self._evaluate()

    def _on_compatibility(self, msg) -> None:
        self.compatible = bool(msg.permit_navigation and msg.observation_ready)
        self._evaluate()

    def _on_host(self, msg) -> None:
        # WAIT_FRESH_HOST_INTENT is intentionally admissible: the newly
        # accepted public goal is the fresh intent that creates the next Host
        # command epoch.  All earlier quiet/wire gates remain closed.
        self.host_safe = bool(
            msg.control_state == HostControlState.STATE_WAIT_FRESH_HOST_INTENT
            or (msg.control_state == HostControlState.STATE_HOST_ACTIVE and not msg.rearm_required)
        )
        self._evaluate()

    def _evaluate(self) -> None:
        flags = 0
        if not self.link_safe:
            flags |= NavigationGuardState.REASON_LINK
        if not self.firmware_safe:
            flags |= NavigationGuardState.REASON_FIRMWARE
        if not self.supervision_safe:
            flags |= NavigationGuardState.REASON_SUPERVISION
        if not self.compatible:
            flags |= NavigationGuardState.REASON_COMPATIBILITY
        if not self.host_safe:
            flags |= NavigationGuardState.REASON_HOST_CONTROL
        self.reason_flags = flags
        if flags == 0:
            self.guard.recover()
        else:
            self.guard.revoke()
        self._publish()

    def _goal(self, _request):
        return GoalResponse.ACCEPT if self.guard.ready else GoalResponse.REJECT

    def _cancel(self, _goal):
        self.guard.revoke()
        self._publish()
        return CancelResponse.ACCEPT

    async def _execute(self, goal):
        uuid = bytes(goal.goal_id.uuid).hex()
        if not self.guard.observe_goal(uuid):
            goal.abort()
            return NavigateToPose.Result()
        self._publish()
        if not self.backend.wait_for_server(timeout_sec=1.0):
            self.guard.revoke()
            self.guard.terminate(uuid)
            goal.abort()
            self._publish()
            return NavigateToPose.Result()
        send_future = self.backend.send_goal_async(
            goal.request,
            feedback_callback=lambda feedback: goal.publish_feedback(feedback.feedback),
        )
        self.backend_handle = await send_future
        if self.backend_handle is None or not self.backend_handle.accepted:
            self.guard.terminate(uuid)
            goal.abort()
            self._publish()
            return NavigateToPose.Result()
        result_future = self.backend_handle.get_result_async()
        while not result_future.done():
            if goal.is_cancel_requested:
                self.guard.revoke()
            self._cancel_tick()
            await asyncio.sleep(0.02)
        wrapped = result_future.result()
        self.backend_handle = None
        was_revoked = self.guard.cancel_pending or uuid == self.guard.revoked_uuid
        self.guard.terminate(uuid)
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and not was_revoked:
            goal.succeed()
        elif wrapped.status == GoalStatus.STATUS_CANCELED or goal.is_cancel_requested:
            goal.canceled()
        else:
            goal.abort()
        self._publish()
        return wrapped.result

    def _cancel_tick(self) -> None:
        if not self.guard.cancel_pending or self.backend_handle is None:
            return
        now = time.monotonic()
        if self.last_cancel_at is not None and now - self.last_cancel_at < self.cancel_retry_sec:
            return
        self.last_cancel_at = now
        self.guard.note_cancel_attempt()
        self.backend_handle.cancel_goal_async()
        self._publish()

    def _publish(self) -> None:
        msg = NavigationGuardState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.guard.cancel_pending:
            msg.state = (
                NavigationGuardState.STATE_CANCEL_PENDING
                if self.guard.cancel_attempts == 0
                else NavigationGuardState.STATE_WAIT_OLD_GOAL_TERMINATED
            )
        elif self.guard.active_uuid:
            msg.state = NavigationGuardState.STATE_ACTIVE
        elif self.guard.ready:
            msg.state = NavigationGuardState.STATE_READY_FOR_NEW_GOAL
        else:
            msg.state = NavigationGuardState.STATE_CLOSED
        msg.revoke_generation = self.guard.revoke_generation
        msg.goal_generation = self.guard.goal_generation
        msg.cancel_attempts = self.guard.cancel_attempts
        msg.reason_flags = self.reason_flags
        msg.active_goal_uuid = self.guard.active_uuid
        msg.revoked_goal_uuid = self.guard.revoked_uuid
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationGuardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
