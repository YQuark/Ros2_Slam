#!/usr/bin/env python3
"""ROS adapter for motion supervision decisions."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import HostMotionCommand, MotionSupervisionState, WheelObservation
from sensor_msgs.msg import Imu
from robot_chassis_model.wheel_layout import WheelLayout, WheelLayoutError

from robot_supervision.motion_supervisor import (
    MotionSupervisor,
    SupervisorConfig,
    SupervisorLevel,
    observation_requires_release,
)


class MotionSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("motion_supervisor")
        defaults = SupervisorConfig()
        self.declare_parameter("config_sha256", "development-uncompiled")
        self.declare_parameter("wheel_observation_topic", "wheel/observation")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("host_motion_command_topic", "chassis/host_motion_command")
        self.declare_parameter("motion_supervision_topic", "motion/supervision_state")
        self.declare_parameter("wheel_track_width", 0.176)
        self.declare_parameter("observation_timeout_sec", 0.25)
        self.declare_parameter("imu_timeout_sec", 0.20)
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        values = {name: self.get_parameter(name).value for name in defaults.__dataclass_fields__}
        self.supervisor = MotionSupervisor(SupervisorConfig(**values))
        self.track_width = max(float(self.get_parameter("wheel_track_width").value), 1e-6)
        self.observation_timeout = float(self.get_parameter("observation_timeout_sec").value)
        self.imu_timeout = float(self.get_parameter("imu_timeout_sec").value)
        self.config_sha256 = str(self.get_parameter("config_sha256").value)
        self.last_command = None
        self.last_gyro_z = None
        self.last_imu_at = None
        self.last_observation_at = None
        self.last_observation = None
        self.last_sample_identity = None
        self.publisher = self.create_publisher(
            MotionSupervisionState,
            str(self.get_parameter("motion_supervision_topic").value),
            10,
        )
        self.create_subscription(
            HostMotionCommand,
            str(self.get_parameter("host_motion_command_topic").value),
            self._on_command,
            10,
        )
        self.create_subscription(
            WheelObservation,
            str(self.get_parameter("wheel_observation_topic").value),
            self._on_wheel,
            20,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self._health_tick)

    def _on_command(self, msg: HostMotionCommand) -> None:
        self.last_command = msg

    def _on_imu(self, msg: Imu) -> None:
        self.last_gyro_z = float(msg.angular_velocity.z)
        self.last_imu_at = time.monotonic()

    def _on_wheel(self, msg: WheelObservation) -> None:
        now = time.monotonic()
        identity = (int(msg.transport_session_id), int(msg.status_sequence))
        if self.last_sample_identity is not None:
            old_session, old_sequence = self.last_sample_identity
            delta = (identity[1] - old_sequence) & 0xFFFFFFFF
            if identity[0] == old_session and not (0 < delta < 0x80000000):
                return
        self.last_sample_identity = identity
        self.last_observation_at = now
        self.last_observation = msg
        if observation_requires_release(
            schema_version=msg.schema_version,
            expected_schema_version=WheelObservation.SCHEMA_VERSION,
            encoder_anomaly_mask=msg.encoder_anomaly_mask,
            enabled_mask=msg.motor_enabled_mask,
            speed_valid_mask=msg.speed_valid_mask,
        ):
            self._publish(
                SupervisorLevel.CRITICAL,
                1.0,
                0.0,
                True,
                MotionSupervisionState.REASON_OBSERVATION_INVALID,
            )
            return
        speeds = tuple(float(value) for value in msg.wheel_speed_mps)
        targets = tuple(float(value) for value in msg.wheel_target_mps)
        layout = WheelLayout(msg.motor_enabled_mask, msg.speed_valid_mask, msg.encoder_anomaly_mask)
        try:
            left, right = layout.aggregate(speeds)
        except WheelLayoutError:
            self._publish(
                SupervisorLevel.CRITICAL,
                1.0,
                0.0,
                True,
                MotionSupervisionState.REASON_OBSERVATION_INVALID,
            )
            return
        command_vx = 0.0
        command_wz = 0.0
        if self.last_command is not None and self.last_command.enable:
            command_vx = float(self.last_command.twist.linear.x)
            command_wz = float(self.last_command.twist.angular.z)
        gyro = (
            self.last_gyro_z
            if self.last_imu_at is not None and now - self.last_imu_at <= self.imu_timeout
            else None
        )
        result = self.supervisor.update(
            now_sec=now,
            command_vx=command_vx,
            command_wz=command_wz,
            wheel_speeds=speeds,
            wheel_targets=targets,
            enabled_mask=msg.motor_enabled_mask,
            speed_valid_mask=msg.speed_valid_mask,
            anomaly_mask=msg.encoder_anomaly_mask,
            feedback_vx=0.5 * (left + right),
            wheel_wz=(right - left) / self.track_width,
            gyro_z=gyro,
        )
        reason_map = {
            "wheel_pair": MotionSupervisionState.REASON_WHEEL_PAIR,
            "tracking": MotionSupervisionState.REASON_TRACKING,
            "yaw": MotionSupervisionState.REASON_YAW_DISAGREEMENT,
            "unexpected_motion": MotionSupervisionState.REASON_UNEXPECTED_MOTION,
        }
        reason_flags = reason_map.get(result.reason, MotionSupervisionState.REASON_NONE)
        if gyro is None:
            reason_flags |= MotionSupervisionState.REASON_IMU_UNAVAILABLE
        self._publish(
            result.level,
            result.score,
            result.command_scale,
            result.release_host_candidate,
            reason_flags,
        )

    def _health_tick(self) -> None:
        now = time.monotonic()
        if (
            self.last_observation_at is None
            or now - self.last_observation_at > self.observation_timeout
        ):
            self._publish(
                SupervisorLevel.CRITICAL,
                1.0,
                0.0,
                True,
                MotionSupervisionState.REASON_OBSERVATION_STALE,
            )

    def _publish(self, level, score, scale, release, reason_flags) -> None:
        msg = MotionSupervisionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.level = int(level) + 1
        msg.reason_flags = int(reason_flags)
        msg.score = float(score)
        msg.command_scale = float(scale)
        msg.release_host_candidate = bool(release)
        if self.last_command is not None:
            msg.command_epoch = self.last_command.command_epoch
            msg.command_sequence = self.last_command.sequence
        if self.last_observation is not None:
            msg.observation_session_id = self.last_observation.transport_session_id
            msg.observation_sequence = self.last_observation.status_sequence
        msg.observation_age_ms = (
            0xFFFFFFFF
            if self.last_observation_at is None
            else min(int(max(time.monotonic() - self.last_observation_at, 0.0) * 1000), 0xFFFFFFFF)
        )
        msg.config_sha256 = self.config_sha256
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
