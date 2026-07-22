#!/usr/bin/env python3
import secrets
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from robot_interfaces.msg import ChassisCommand, ChassisState, ControlState, MotionSafetyState

from robot_control.control_policy import (
    Command,
    CommandMux,
    InvalidCommandError,
    MotionLimiter,
    SelectedCommand,
    SourceConfig,
)


class CmdVelMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")

        self.declare_parameter("config_sha256", "development-uncompiled")
        self.declare_parameter("unsafe_development_mode", True)
        self.declare_parameter("research_sources", [])
        self.declare_parameter("linear_limit", 0.4)
        self.declare_parameter("angular_limit", 1.5)
        self.declare_parameter("input_linear_abs_max", 5.0)
        self.declare_parameter("input_angular_abs_max", 20.0)
        self.declare_parameter("max_linear_accel", 0.5)
        self.declare_parameter("max_angular_accel", 1.5)
        self.declare_parameter("max_linear_jerk", 2.0)
        self.declare_parameter("max_angular_jerk", 6.0)
        self.declare_parameter("motion_limiter_max_dt", 0.1)
        self.declare_parameter("timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("chassis_command_topic", "chassis/command")
        self.declare_parameter("publish_legacy_twist", False)
        self.declare_parameter("legacy_driver_topic", "cmd_vel/driver")
        self.declare_parameter("motion_safety_topic", "motion/safety_state")
        self.declare_parameter("chassis_state_topic", "chassis/state")
        self.declare_parameter("control_state_topic", "chassis/control_state")
        self.declare_parameter("require_motion_supervision", True)
        self.declare_parameter("supervision_timeout_sec", 0.25)
        self.declare_parameter("rearm_quiet_sec", 0.25)

        research_sources = [str(value) for value in self.get_parameter("research_sources").value]
        self.source_config = SourceConfig(research_sources=research_sources)
        self.mux = CommandMux(
            source_config=self.source_config,
            linear_limit=float(self.get_parameter("linear_limit").value),
            angular_limit=float(self.get_parameter("angular_limit").value),
            timeout_sec=float(self.get_parameter("timeout_sec").value),
            input_linear_abs_max=float(self.get_parameter("input_linear_abs_max").value),
            input_angular_abs_max=float(self.get_parameter("input_angular_abs_max").value),
        )

        self.chassis_command_topic = str(self.get_parameter("chassis_command_topic").value)
        self.publisher = self.create_publisher(ChassisCommand, self.chassis_command_topic, 10)
        self.state_publisher = self.create_publisher(
            ControlState, str(self.get_parameter("control_state_topic").value), 10
        )
        self.publish_legacy_twist = bool(self.get_parameter("publish_legacy_twist").value)
        self.legacy_driver_topic = str(self.get_parameter("legacy_driver_topic").value)
        self.legacy_publisher = (
            self.create_publisher(Twist, self.legacy_driver_topic, 10)
            if self.publish_legacy_twist
            else None
        )
        self.sequence = 0
        self.session_id = 0
        self.motion_limiter = MotionLimiter(
            max_linear_accel=float(self.get_parameter("max_linear_accel").value),
            max_angular_accel=float(self.get_parameter("max_angular_accel").value),
            max_linear_jerk=float(self.get_parameter("max_linear_jerk").value),
            max_angular_jerk=float(self.get_parameter("max_angular_jerk").value),
            max_dt_sec=float(self.get_parameter("motion_limiter_max_dt").value),
            max_linear_velocity=float(self.get_parameter("linear_limit").value),
            max_angular_velocity=float(self.get_parameter("angular_limit").value),
        )
        self.last_active = False
        self.last_source = "idle"
        self.invalid_command_count = 0
        self.config_sha256 = str(self.get_parameter("config_sha256").value)
        self.require_supervision = bool(self.get_parameter("require_motion_supervision").value)
        self.supervision_timeout = float(self.get_parameter("supervision_timeout_sec").value)
        self.rearm_quiet_sec = float(self.get_parameter("rearm_quiet_sec").value)
        self.motion_state = None
        self.motion_state_at = None
        self.motion_scale = 1.0
        self.supervision_seen = False
        self.rearm_required = False
        self.rearm_reason_flags = 0
        self.gate_state = ControlState.STATE_WAIT_SUPERVISION
        self.last_reject_reason = ControlState.REJECT_NONE
        self.last_wire_session_id = 0
        self.release_sent_wire_session = None
        self.subscriptions_by_source: Dict[str, object] = {}
        for source, topic in self.source_config.topic_map().items():
            self.subscriptions_by_source[source] = self.create_subscription(
                Twist,
                topic,
                self._make_callback(source),
                10,
            )
        self.create_subscription(
            MotionSafetyState,
            str(self.get_parameter("motion_safety_topic").value),
            self._on_motion_state,
            10,
        )
        self.create_subscription(
            ChassisState,
            str(self.get_parameter("chassis_state_topic").value),
            self._on_chassis_state,
            10,
        )

        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(1.0 / publish_hz, self._publish_selected)
        self.get_logger().info(
            "cmd_vel_mux active: "
            f"sources={','.join(self.source_config.topic_map().keys())} "
            f"chassis_command_topic={self.chassis_command_topic} "
            f"legacy_twist={1 if self.publish_legacy_twist else 0}"
        )

    def _make_callback(self, source: str):
        def _callback(msg: Twist) -> None:
            try:
                self.mux.update(
                    source,
                    Command(linear_x=msg.linear.x, angular_z=msg.angular.z),
                    self.get_clock().now().nanoseconds * 1e-9,
                )
            except (InvalidCommandError, TypeError, ValueError) as exc:
                self.invalid_command_count += 1
                self.last_reject_reason = ControlState.REJECT_INVALID
                self.get_logger().error(f"Rejected {source} command: {exc}")
                self._publish_release()
                return

            if self.rearm_required:
                if self.gate_state == ControlState.STATE_WAIT_FRESH_SOURCE:
                    self.rearm_required = False
                    self.rearm_reason_flags = 0
                    self.gate_state = ControlState.STATE_READY
                    self.last_active = False
                    self.session_id = 0
                    self.sequence = 0
                    self.motion_limiter.reset()
                else:
                    return
            self.last_reject_reason = ControlState.REJECT_NONE
            if not self.last_active:
                self._publish_selected()

        return _callback

    def _publish_selected(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.require_supervision:
            supervision_fresh = (
                self.motion_state_at is not None
                and 0.0 <= now_sec - self.motion_state_at <= self.supervision_timeout
            )
            if not supervision_fresh:
                if self.supervision_seen:
                    self._latch_rearm(
                        ControlState.REARM_SUPERVISION_STALE,
                        ControlState.REJECT_SUPERVISION_STALE,
                    )
                elif not self.rearm_required:
                    self.gate_state = ControlState.STATE_WAIT_SUPERVISION
                self._publish_control_state(now_sec)
                return
        if self.rearm_required:
            if (
                self.gate_state == ControlState.STATE_WAIT_SOURCE_QUIET
                and not self.mux.has_recent_input(now_sec, self.rearm_quiet_sec)
            ):
                self.mux.clear()
                self.gate_state = ControlState.STATE_WAIT_FRESH_SOURCE
            self._publish_control_state(now_sec)
            return
        selected = self.mux.select(now_sec)
        if not selected.active:
            if self.last_active:
                self._publish_release()
            self.gate_state = ControlState.STATE_READY
            self._publish_control_state(now_sec)
            return

        if not self.last_active:
            self.session_id = secrets.randbits(64) or 1
            self.sequence = 0

        scaled = Command(
            selected.command.linear_x * self.motion_scale,
            selected.command.angular_z * self.motion_scale,
        )
        limited = self.motion_limiter.limit(scaled, now_sec=now_sec)
        self._publish_command(SelectedCommand(source=selected.source, command=limited, active=True))
        self.last_active = True
        self.last_source = selected.source
        self.gate_state = ControlState.STATE_ACTIVE
        self._publish_control_state(now_sec)

    def _publish_release(self) -> None:
        if self.session_id == 0:
            self.session_id = secrets.randbits(64) or 1
        self.motion_limiter.reset()
        selected = SelectedCommand(source="idle", command=Command(0.0, 0.0), active=False)
        self._publish_command(selected)
        self.last_active = False
        self.last_source = "idle"

    def _on_motion_state(self, msg: MotionSafetyState) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.motion_state = msg
        self.motion_state_at = now_sec
        self.supervision_seen = True
        self.motion_scale = max(0.0, min(float(msg.command_scale), 1.0))
        if msg.release_required or msg.level == MotionSafetyState.LEVEL_CRITICAL:
            self._latch_rearm(
                ControlState.REARM_MOTION_CRITICAL,
                ControlState.REJECT_MOTION_CRITICAL,
            )
        elif not self.rearm_required:
            self.gate_state = ControlState.STATE_READY

    def _on_chassis_state(self, msg: ChassisState) -> None:
        wire_session = int(msg.wire_session_id)
        session_changed = wire_session != 0 and wire_session != self.last_wire_session_id
        self.last_wire_session_id = wire_session
        if msg.rearm_required:
            self._latch_rearm(
                int(msg.rearm_reason_flags) or ControlState.REARM_TRANSPORT,
                ControlState.REJECT_REARM_REQUIRED,
                force_release=session_changed,
            )

    def _latch_rearm(self, reason_flags, reject_reason, *, force_release=False) -> None:
        first = not self.rearm_required
        self.rearm_required = True
        self.rearm_reason_flags |= int(reason_flags)
        self.last_reject_reason = int(reject_reason)
        if first:
            self.mux.clear()
            self.motion_limiter.reset()
            self.gate_state = ControlState.STATE_WAIT_SOURCE_QUIET
        if first or force_release:
            self._publish_release()
            self.release_sent_wire_session = self.last_wire_session_id

    def _publish_control_state(self, now_sec: float) -> None:
        msg = ControlState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.control_state = self.gate_state
        msg.selected_source = self._source_id(self.last_source)
        age = self.mux.newest_age(now_sec)
        msg.command_age_ms = 0xFFFFFFFF if age is None else min(int(age * 1000.0), 0xFFFFFFFF)
        msg.command_reject_reason = self.last_reject_reason
        msg.enable_intent = self.last_active and not self.rearm_required
        msg.rearm_required = self.rearm_required
        msg.rearm_reason_flags = self.rearm_reason_flags
        msg.command_session_id = self.session_id
        msg.command_sequence = self.sequence
        msg.config_sha256 = self.config_sha256
        self.state_publisher.publish(msg)

    def _publish_command(self, selected) -> None:
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        msg = ChassisCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = selected.command.linear_x
        msg.twist.angular.z = selected.command.angular_z
        msg.enable = selected.active
        msg.source = self._source_id(selected.source)
        msg.session_id = self.session_id
        msg.sequence = self.sequence
        self.publisher.publish(msg)

        if self.legacy_publisher is not None:
            legacy = Twist()
            legacy.linear.x = selected.command.linear_x
            legacy.angular.z = selected.command.angular_z
            self.legacy_publisher.publish(legacy)

    @staticmethod
    def _source_id(source: str) -> int:
        if source == "teleop":
            return ChassisCommand.SOURCE_TELEOP
        if source == "test":
            return ChassisCommand.SOURCE_TEST
        if source == "nav":
            return ChassisCommand.SOURCE_NAV
        if source.startswith("research/"):
            return ChassisCommand.SOURCE_RESEARCH
        return ChassisCommand.SOURCE_NONE


def main() -> None:
    rclpy.init()
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
