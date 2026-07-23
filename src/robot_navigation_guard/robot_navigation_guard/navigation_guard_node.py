#!/usr/bin/env python3
"""Cancel Nav2 goals on chassis revocation and require a new goal UUID."""

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from robot_interfaces.msg import (
    ChassisLinkState,
    FirmwareControlState,
    MotionSupervisionState,
    NavigationGuardState,
    PlatformCompatibilityState,
)

from robot_navigation_guard.guard import GoalGuard


class NavigationGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_guard")
        self.guard = GoalGuard()
        self.link_safe = self.firmware_safe = self.supervision_safe = self.compatible = False
        self.cancel_client = self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")
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
        self.create_subscription(
            GoalStatusArray, "navigate_to_pose/_action/status", self._on_status, 10
        )

    def _on_link(self, msg) -> None:
        self.link_safe = (
            msg.link_state == ChassisLinkState.STATE_WIRE_SYNCHRONIZED
            and not msg.wire_rearm_required
        )
        self._evaluate(1)

    def _on_firmware(self, msg) -> None:
        self.firmware_safe = not msg.estop_active and not msg.fault_stop_active
        self._evaluate(2)

    def _on_supervision(self, msg) -> None:
        self.supervision_safe = (
            msg.level != MotionSupervisionState.LEVEL_CRITICAL and not msg.release_host_candidate
        )
        self._evaluate(4)

    def _on_compatibility(self, msg) -> None:
        self.compatible = bool(msg.permit_navigation)
        self._evaluate(8)

    def _on_status(self, msg) -> None:
        active = next(
            (
                item
                for item in msg.status_list
                if item.status
                in (
                    GoalStatus.STATUS_ACCEPTED,
                    GoalStatus.STATUS_EXECUTING,
                    GoalStatus.STATUS_CANCELING,
                )
            ),
            None,
        )
        if active is None:
            self.guard.active_uuid = ""
        else:
            uuid = bytes(active.goal_info.goal_id.uuid).hex()
            if not self.guard.observe_goal(uuid):
                self._cancel_all()
        self._publish(0)

    def _evaluate(self, reason) -> None:
        if self.link_safe and self.firmware_safe and self.supervision_safe and self.compatible:
            self.guard.recover()
        elif self.guard.revoke():
            self._cancel_all()
        self._publish(reason)

    def _cancel_all(self) -> None:
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())

    def _publish(self, reason) -> None:
        msg = NavigationGuardState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = (
            NavigationGuardState.STATE_ACTIVE
            if self.guard.active_uuid
            else (
                NavigationGuardState.STATE_READY_FOR_NEW_GOAL
                if self.guard.safe
                else NavigationGuardState.STATE_CLOSED
            )
        )
        msg.revoke_generation = self.guard.generation
        msg.reason_flags = reason
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
