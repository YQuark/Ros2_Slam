#!/usr/bin/env python3
"""ROS adapter for the fail-closed upper protocol v3 bridge."""

from __future__ import annotations

import math
import secrets
import time
from dataclasses import replace

import rclpy
from builtin_interfaces.msg import Time as TimeMessage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from robot_interfaces.msg import ChassisCommand, ChassisState, FirmwareInfo
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float32, UInt32
from std_srvs.srv import SetBool, Trigger

try:
    import serial
except Exception:  # pragma: no cover
    serial = None

from stm32_robot_bridge.bridge_core import BridgeCore
from stm32_robot_bridge.imu_converter import (
    AffineClockSynchronizer,
    accel_g_to_mps2,
    classify_imu_quality,
    gyro_dps_to_rad,
    normalize_quaternion,
)
from stm32_robot_bridge.motion_supervisor import MotionSupervisor, SupervisorConfig, SupervisorLevel
from stm32_robot_bridge.odometry import EncoderOdometry
from stm32_robot_bridge.protocol_v3 import (
    ACK_APPLIED,
    ACK_RECEIVED,
    ACK_REJECTED,
    CMD_CLEAR_FAULT,
    CMD_DIAGNOSTIC,
    CMD_ESTOP,
    CMD_GET_INFO,
    CMD_HELLO,
    CMD_IMU_STATUS,
    CMD_LINE_CTRL,
    CMD_SET_VELOCITY,
    CMD_STATUS,
    IMU_FLAG_ERROR,
    IMU_FLAG_ONLINE,
    PROTOCOL_VERSION,
    CommandStream,
    decode_diagnostic_payload,
    decode_hello_payload,
    decode_imu_status_payload,
    decode_status_payload,
    encode_clear_fault_payload,
    encode_estop_payload,
    encode_get_info_payload,
    encode_line_ctrl_payload,
)
from stm32_robot_bridge.transport_supervisor import SerialTransportSupervisor


def _qos_command(deadline_sec: float) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=deadline_sec),
    )


STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.RELIABLE
)
STATIC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class STM32BridgeV3(Node):
    def __init__(self) -> None:
        super().__init__("stm32_bridge")
        defaults = {
            "config_sha256": "development-uncompiled",
            "unsafe_development_mode": True,
            "port": "/dev/serial0",
            "baudrate": 115200,
            "protocol_version": 3,
            "chassis_command_topic": "chassis/command",
            "odom_topic": "wheel/odom",
            "imu_topic": "imu/data",
            "frame_id": "odom",
            "child_frame_id": "base_link",
            "imu_frame_id": "imu_link",
            "publish_tf": False,
            "cmd_timeout": 0.15,
            "drive_keepalive_sec": 0.05,
            "status_timeout": 0.25,
            "command_ack_timeout_sec": 0.15,
            "max_command_age_sec": 0.15,
            "hard_max_linear_mps": 0.45,
            "hard_max_angular_radps": 1.5,
            "wheel_radius": 0.035,
            "wheel_track_width": 0.176,
            "encoder_counts_per_revolution": 2464.0,
            "odom_linear_scale": 1.0,
            "odom_angular_scale": 1.0,
            "odom_angular_sign": 1.0,
            "odom_covariance.wheel_variance_floor_m2": 0.000025,
            "odom_covariance.wheel_variance_per_meter": 0.0025,
            "imu.use_orientation": False,
            "imu.orientation_stddev": 0.2,
            "imu.angular_velocity_stddev": [0.02, 0.02, 0.02],
            "imu.linear_acceleration_stddev": [0.2, 0.2, 0.2],
        }
        supervisor_defaults = SupervisorConfig()
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in supervisor_defaults.__dataclass_fields__:
            self.declare_parameter(f"motion_supervisor.{name}", getattr(supervisor_defaults, name))
        if int(self.get_parameter("protocol_version").value) != 3:
            raise RuntimeError("STM32 bridge v0.4.0 only supports upper protocol v3")
        self.config_sha256 = str(self.get_parameter("config_sha256").value)
        self.status_timeout = float(self.get_parameter("status_timeout").value)
        self.ack_timeout = float(self.get_parameter("command_ack_timeout_sec").value)
        self.imu_use_orientation = bool(self.get_parameter("imu.use_orientation").value)
        self.imu_orientation_stddev = float(self.get_parameter("imu.orientation_stddev").value)
        self.imu_gyro_stddev = tuple(
            float(v) for v in self.get_parameter("imu.angular_velocity_stddev").value
        )
        self.imu_accel_stddev = tuple(
            float(v) for v in self.get_parameter("imu.linear_acceleration_stddev").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        if bool(self.get_parameter("publish_tf").value):
            raise RuntimeError("bridge v3 never owns odom->base_link TF")

        self.core = BridgeCore(
            hard_max_linear_mps=float(self.get_parameter("hard_max_linear_mps").value),
            hard_max_angular_radps=float(self.get_parameter("hard_max_angular_radps").value),
            command_timeout_sec=float(self.get_parameter("cmd_timeout").value),
            status_timeout_sec=self.status_timeout,
            max_command_age_sec=float(self.get_parameter("max_command_age_sec").value),
        )
        self.command_stream = CommandStream(
            float(self.get_parameter("cmd_timeout").value),
            float(self.get_parameter("drive_keepalive_sec").value),
            secrets.randbits(64) or 1,
        )
        self.odometry = EncoderOdometry(
            wheel_radius_m=float(self.get_parameter("wheel_radius").value),
            track_width_m=float(self.get_parameter("wheel_track_width").value),
            counts_per_revolution=float(self.get_parameter("encoder_counts_per_revolution").value),
            linear_scale=float(self.get_parameter("odom_linear_scale").value),
            angular_scale=float(self.get_parameter("odom_angular_scale").value),
            angular_sign=float(self.get_parameter("odom_angular_sign").value),
            wheel_variance_floor_m2=float(
                self.get_parameter("odom_covariance.wheel_variance_floor_m2").value
            ),
            wheel_variance_per_meter=float(
                self.get_parameter("odom_covariance.wheel_variance_per_meter").value
            ),
        )
        supervisor_values = {
            name: self.get_parameter(f"motion_supervisor.{name}").value
            for name in supervisor_defaults.__dataclass_fields__
        }
        self.supervisor = MotionSupervisor(SupervisorConfig(**supervisor_values))
        self.supervisor_result = self.supervisor.update(
            now_sec=0.0,
            command_vx=0.0,
            command_wz=0.0,
            wheel_speeds=(0.0,) * 4,
            wheel_targets=(0.0,) * 4,
            feedback_vx=0.0,
            wheel_wz=0.0,
            gyro_z=None,
        )
        self.status_clock_sync = AffineClockSynchronizer()
        self.imu_clock_sync = AffineClockSynchronizer()
        self.last_gyro_z = None
        self.last_imu_monotonic = None
        self.last_imu_quality = 0
        self.invalid_imu_count = 0
        self.last_ack_progress = time.monotonic()
        self.last_applied_sequence = 0
        self.release_count = 0
        self.invalid_command_count = 0

        if serial is None:
            raise RuntimeError("python3-serial is required")
        self.transport = SerialTransportSupervisor(
            serial, str(self.get_parameter("port").value), int(self.get_parameter("baudrate").value)
        )

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), STATE_QOS
        )
        self.imu_pub = self.create_publisher(
            Imu, str(self.get_parameter("imu_topic").value), qos_profile_sensor_data
        )
        self.state_pub = self.create_publisher(ChassisState, "chassis/state", STATE_QOS)
        self.firmware_pub = self.create_publisher(FirmwareInfo, "chassis/firmware_info", STATIC_QOS)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "diagnostics", STATE_QOS)
        self.chassis_status_pub = self.create_publisher(UInt32, "chassis/status", STATE_QOS)
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", STATE_QOS)
        self.left_current_pub = self.create_publisher(Float32, "motor/left_current", STATE_QOS)
        self.right_current_pub = self.create_publisher(Float32, "motor/right_current", STATE_QOS)
        self.create_subscription(
            ChassisCommand,
            str(self.get_parameter("chassis_command_topic").value),
            self.on_chassis_command,
            _qos_command(float(self.get_parameter("cmd_timeout").value)),
        )
        self.create_service(SetBool, "chassis/estop", self.on_estop)
        self.create_service(Trigger, "chassis/clear_fault", self.on_clear_fault)
        self.create_service(SetBool, "chassis/line_ctrl", self.on_line_ctrl)
        self.create_timer(0.01, self.control_tick)
        self.create_timer(1.0, self.publish_diagnostics)
        self.transport.start()
        self.get_logger().warn(
            "upper protocol v3 only; beta4/v2 firmware is intentionally incompatible"
        )

    def on_chassis_command(self, msg: ChassisCommand) -> None:
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        now_mono = time.monotonic()
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        try:
            command = self.core.accept_command(
                vx=msg.twist.linear.x,
                wz=msg.twist.angular.z,
                enable=bool(msg.enable),
                source=int(msg.source),
                session_id=int(msg.session_id),
                sequence=int(msg.sequence),
                command_stamp_sec=stamp,
                now_ros_sec=now_ros,
                now_monotonic=now_mono,
            )
        except ValueError as exc:
            self.invalid_command_count += 1
            self._release(f"rejected command: {exc}")
            return
        if not command.enable:
            self._release("command disabled")
            return
        scale = self.supervisor_result.command_scale
        if not self.command_stream.enabled:
            self.last_ack_progress = now_mono
        self.command_stream.update_command(command.vx * scale, command.wz * scale, now_mono)

    def control_tick(self) -> None:
        for event in self.transport.drain_events():
            if event.kind == "connected":
                self._on_connected()
            elif event.kind == "disconnected":
                self.core.on_disconnected()
                self.odometry.reset_sample_baseline()
                self.status_clock_sync.reset()
                self.imu_clock_sync.reset()
            elif event.kind == "frame":
                self._handle_frame(event.cmd, event.payload, event.received_at)
        now = time.monotonic()
        release, reason = self.core.tick(now)
        if release:
            self._release(reason)
        if self.core.machine.can_drive:
            self.command_stream.tick(
                now, lambda payload: self.transport.submit(CMD_SET_VELOCITY, payload, coalesce=True)
            )
            self.core.snapshot = replace(
                self.core.snapshot, wire_sent_sequence=self.command_stream.sequence
            )
        status = self.core.snapshot.firmware.status if self.core.snapshot.firmware else None
        ack_current = bool(
            status
            and status.last_received_session_id == self.command_stream.session_id
            and status.last_received_sequence == self.command_stream.sequence
            and status.command_ack_flags & (ACK_RECEIVED | ACK_APPLIED)
            and not status.command_ack_flags & ACK_REJECTED
        )
        if self.command_stream.enabled and ack_current:
            self.last_applied_sequence = status.last_applied_sequence
            self.last_ack_progress = now
        if self.command_stream.enabled and now - self.last_ack_progress > self.ack_timeout:
            self._release("command ACK timeout")

    def _on_connected(self) -> None:
        self._begin_wire_session(request_info=True)

    def _begin_wire_session(self, *, request_info: bool) -> None:
        session = secrets.randbits(64) or 1
        self.core.on_connected(session)
        self.command_stream = CommandStream(
            float(self.get_parameter("cmd_timeout").value),
            float(self.get_parameter("drive_keepalive_sec").value),
            session,
        )
        self.last_ack_progress = time.monotonic()
        if request_info:
            self.transport.submit(CMD_GET_INFO, encode_get_info_payload(), critical=True)
        for _ in range(3):
            self.command_stream.release(
                lambda payload: self.transport.submit(CMD_SET_VELOCITY, payload, critical=True)
            )
        self.core.snapshot = replace(
            self.core.snapshot, wire_sent_sequence=self.command_stream.sequence
        )
        self.core.on_startup_released()

    def _handle_frame(self, cmd: int, payload: bytes, received_at: float) -> None:
        if cmd == CMD_HELLO:
            hello = decode_hello_payload(payload)
            if hello and self.core.snapshot.hello is not None:
                self.odometry.reset_sample_baseline()
                self.status_clock_sync.reset()
                self.imu_clock_sync.reset()
                self._begin_wire_session(request_info=False)
            if hello and self.core.on_hello(hello):
                msg = FirmwareInfo()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.protocol_version, msg.schema_version = hello.version, hello.schema_version
                msg.capabilities, msg.firmware_commit = hello.capabilities, hello.firmware_commit
                msg.hardware_revision, msg.parameter_crc32 = (
                    hello.hardware_revision,
                    hello.parameter_crc32,
                )
                self.firmware_pub.publish(msg)
            return
        if cmd == CMD_STATUS:
            status = decode_status_payload(payload)
            if status is not None and self.core.on_status(status, received_at):
                self._publish_status(status, received_at)
            return
        if cmd == CMD_IMU_STATUS:
            imu = decode_imu_status_payload(payload)
            if imu is not None:
                self._publish_imu(imu)
            return
        if cmd == CMD_DIAGNOSTIC:
            decode_diagnostic_payload(payload)

    def _publish_status(self, status, received_at: float) -> None:
        if self.command_stream.enabled and (
            status.last_reject_reason != 0 or status.command_ack_flags & ACK_REJECTED
        ):
            self.core.snapshot = replace(self.core.snapshot, rearm_required=True)
            self._release(f"firmware rejected command: {status.last_reject_reason}")
        receive_ros = self.get_clock().now().nanoseconds * 1e-9
        timing = self.status_clock_sync.update(status.sample_timestamp_ms, receive_ros)
        update = self.odometry.update(
            status.encoder_count,
            sample_time_sec=timing.sample_ros_sec,
            valid_mask=status.motor_speed_valid_mask,
            anomaly_mask=status.encoder_anomaly_mask,
            slip_multiplier=self.supervisor_result.covariance_multiplier,
            hard_max_speed_mps=float(self.get_parameter("hard_max_linear_mps").value),
        )
        left_speed = 0.5 * (status.motor_speed_mps[0] + status.motor_speed_mps[1])
        right_speed = 0.5 * (status.motor_speed_mps[2] + status.motor_speed_mps[3])
        feedback_vx = 0.5 * (left_speed + right_speed)
        wheel_wz = (right_speed - left_speed) / max(
            float(self.get_parameter("wheel_track_width").value), 1e-6
        )
        self.supervisor_result = self.supervisor.update(
            now_sec=received_at,
            command_vx=self.core.snapshot.target_vx,
            command_wz=self.core.snapshot.target_wz,
            wheel_speeds=status.motor_speed_mps,
            wheel_targets=status.motor_target_mps,
            feedback_vx=feedback_vx,
            wheel_wz=wheel_wz,
            gyro_z=self.last_gyro_z,
        )
        if self.supervisor_result.release_required:
            self.core.snapshot = replace(self.core.snapshot, rearm_required=True)
            self._release(f"motion supervisor critical: {self.supervisor_result.reason}")
        self._publish_odom(update, timing.sample_ros_sec)
        self.chassis_status_pub.publish(UInt32(data=status.error_flags))
        battery = BatteryState()
        battery.header.stamp = self._time_message(timing.sample_ros_sec)
        battery.voltage, battery.present = status.battery_voltage, status.battery_voltage > 1.0
        self.battery_pub.publish(battery)
        self.left_current_pub.publish(Float32(data=sum(status.motor_current_a[:2])))
        self.right_current_pub.publish(Float32(data=sum(status.motor_current_a[2:])))
        self._publish_chassis_state(status)

    def _publish_odom(self, update, stamp_sec: float) -> None:
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = (
            self._time_message(stamp_sec),
            self.frame_id,
            self.child_frame_id,
        )
        msg.pose.pose.position.x, msg.pose.pose.position.y = update.pose.x, update.pose.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(
            update.pose.yaw / 2.0
        ), math.cos(update.pose.yaw / 2.0)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = update.vx, update.wz
        p = update.pose_covariance
        msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[5] = p[0], p[1], p[2]
        msg.pose.covariance[6], msg.pose.covariance[7], msg.pose.covariance[11] = p[3], p[4], p[5]
        msg.pose.covariance[30], msg.pose.covariance[31], msg.pose.covariance[35] = p[6], p[7], p[8]
        msg.twist.covariance[0] = max(p[0], 1e-6)
        msg.twist.covariance[35] = max(p[8], 1e-6)
        self.odom_pub.publish(msg)

    def _publish_imu(self, imu) -> None:
        validity = classify_imu_quality(
            imu.quality_flags,
            online=bool(imu.status_flags & IMU_FLAG_ONLINE),
            error=bool(imu.status_flags & IMU_FLAG_ERROR),
        )
        if not validity.gyro_valid:
            self.invalid_imu_count += 1
            return
        receive_ros = self.get_clock().now().nanoseconds * 1e-9
        timing = (
            self.imu_clock_sync.update(imu.timestamp_ms, receive_ros)
            if validity.timestamp_valid
            else None
        )
        sample_ros_sec = receive_ros if timing is None else timing.sample_ros_sec
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = (
            self._time_message(sample_ros_sec),
            self.imu_frame_id,
        )
        q = normalize_quaternion(imu.quaternion)
        if not self.imu_use_orientation or not validity.orientation_valid or q is None:
            msg.orientation_covariance[0] = -1.0
        else:
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
            for index in (0, 4, 8):
                msg.orientation_covariance[index] = self.imu_orientation_stddev**2
        gyro = gyro_dps_to_rad(imu.gyro_corrected_dps)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        self.last_gyro_z = gyro[2]
        quality_multiplier = 4.0 if validity.warning else 1.0
        for index, stddev in zip((0, 4, 8), self.imu_gyro_stddev):
            msg.angular_velocity_covariance[index] = stddev**2 * quality_multiplier
        if validity.accel_valid:
            accel = accel_g_to_mps2(imu.accel_g)
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
            for index, stddev in zip((0, 4, 8), self.imu_accel_stddev):
                msg.linear_acceleration_covariance[index] = stddev**2 * quality_multiplier
        else:
            msg.linear_acceleration_covariance[0] = -1.0
        self.last_imu_monotonic, self.last_imu_quality = time.monotonic(), imu.quality_flags
        self.imu_pub.publish(msg)

    def _release(self, reason: str) -> None:
        if self.command_stream.release(
            lambda payload: self.transport.submit(CMD_SET_VELOCITY, payload, critical=True),
            time.monotonic(),
        ):
            self.release_count += 1
            self.core.snapshot = replace(
                self.core.snapshot,
                wire_sent_sequence=self.command_stream.sequence,
                target_vx=0.0,
                target_wz=0.0,
                last_command_at=None,
            )
        if reason:
            self.get_logger().warn(reason)

    def on_estop(self, request, response):
        if not request.data:
            response.success, response.message = False, "remote ESTOP release is forbidden"
            return response
        response.success = self.transport.submit(
            CMD_ESTOP, encode_estop_payload(True), critical=True
        )
        response.message = "estop queued" if response.success else "queue full"
        return response

    def on_clear_fault(self, _request, response):
        response.success = self.transport.submit(
            CMD_CLEAR_FAULT, encode_clear_fault_payload(), critical=True
        )
        response.message = "clear fault queued" if response.success else "queue full"
        return response

    def on_line_ctrl(self, request, response):
        response.success = self.transport.submit(
            CMD_LINE_CTRL, encode_line_ctrl_payload(request.data), critical=True
        )
        response.message = "line control queued" if response.success else "queue full"
        return response

    def _publish_chassis_state(self, status) -> None:
        s = self.core.snapshot
        msg = ChassisState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.bridge_state, msg.selected_source = int(s.state), s.selected_source
        msg.command_session_id, msg.command_sequence = s.command_session_id, s.command_sequence
        msg.upper_enabled = bool(self.command_stream.enabled)
        msg.firmware_control_source, msg.status_flags = status.control_source, status.status_flags
        msg.error_flags, msg.latched_error_flags, msg.protocol_version = (
            status.error_flags,
            status.latched_error_flags,
            3,
        )
        msg.wire_session_id, msg.wire_sent_sequence = (
            s.wire_session_id,
            self.command_stream.sequence,
        )
        msg.firmware_ack_available = True
        msg.firmware_session_id, msg.firmware_received_sequence = (
            status.last_received_session_id,
            status.last_received_sequence,
        )
        msg.firmware_applied_sequence, msg.firmware_reject_reason = (
            status.last_applied_sequence,
            status.last_reject_reason,
        )
        msg.slip_score, msg.supervisor_level = self.supervisor_result.score, int(
            self.supervisor_result.level
        )
        msg.config_sha256 = self.config_sha256
        self.state_pub.publish(msg)

    def publish_diagnostics(self) -> None:
        snapshot = self.core.snapshot
        now = time.monotonic()
        status_age = None if snapshot.last_status_at is None else now - snapshot.last_status_at
        level = DiagnosticStatus.OK
        message = snapshot.state.name.lower()
        if snapshot.hello is None or snapshot.state.value < 3:
            level, message = DiagnosticStatus.ERROR, "v3 HELLO/STATUS/ACK unavailable"
        elif self.supervisor_result.level >= SupervisorLevel.DEGRADED:
            level, message = DiagnosticStatus.WARN, self.supervisor_result.reason
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(
            level=level,
            name="stm32_bridge/v3",
            message=message,
            hardware_id=(snapshot.hello.firmware_commit if snapshot.hello else "unknown"),
        )
        values = {
            "protocol": 3,
            "config_sha256": self.config_sha256,
            "state": snapshot.state.name,
            "status_age": "none" if status_age is None else f"{status_age:.3f}",
            "wire_session": snapshot.wire_session_id,
            "wire_sequence": self.command_stream.sequence,
            "slip_score": f"{self.supervisor_result.score:.3f}",
            "supervisor_level": self.supervisor_result.level.name,
            "release_count": self.release_count,
            "invalid_command_count": self.invalid_command_count,
            "invalid_imu_count": self.invalid_imu_count,
            "status_clock_scale": f"{self.status_clock_sync.scale:.9f}",
            "status_clock_residual_p95": f"{self.status_clock_sync.residual_p95_sec:.6f}",
            "imu_clock_scale": f"{self.imu_clock_sync.scale:.9f}",
            "imu_clock_residual_p95": f"{self.imu_clock_sync.residual_p95_sec:.6f}",
            "command_timeout_sec": self.get_parameter("cmd_timeout").value,
            "status_timeout_sec": self.status_timeout,
            "wheel_radius_m": self.get_parameter("wheel_radius").value,
            "wheel_track_width_m": self.get_parameter("wheel_track_width").value,
            "serial_rx_frames": self.transport.stats.rx_frames,
            "serial_tx_frames": self.transport.stats.tx_frames,
            "serial_crc_errors": self.transport.stats.rx_crc_errors,
            "serial_rx_queue_drops": self.transport.stats.rx_queue_drops,
            "serial_tx_queue_drops": self.transport.stats.tx_queue_drops,
        }
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
        array.status = [status]
        self.diagnostics_pub.publish(array)

    @staticmethod
    def _time_message(seconds: float) -> TimeMessage:
        sec = math.floor(seconds)
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec, nanosec = sec + 1, nanosec - 1_000_000_000
        return TimeMessage(sec=int(sec), nanosec=nanosec)

    def shutdown(self) -> None:
        self._release("bridge shutdown")
        self.transport.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = STM32BridgeV3()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
