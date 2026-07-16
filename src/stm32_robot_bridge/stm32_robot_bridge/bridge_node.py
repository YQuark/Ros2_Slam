#!/usr/bin/env python3
import math
import os
import time
from typing import Dict, Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMessage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_interfaces.msg import ChassisCommand, ChassisState
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float32, UInt32
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

try:
    import serial
except Exception:  # pragma: no cover
    serial = None

try:
    from stm32_robot_bridge.bridge_state import BridgeState, BridgeStateMachine
    from stm32_robot_bridge.command_guard import CommandGuard, CommandRejected
    from stm32_robot_bridge.diagnostics import WarningThrottle
    from stm32_robot_bridge.imu_converter import (
        ImuClockSynchronizer,
        InvalidImuSample,
        accel_g_to_mps2,
        gyro_dps_to_rad,
        normalize_quaternion,
    )
    from stm32_robot_bridge.odometry import (
        DifferentialOdometry,
        DynamicCovarianceModel,
        apply_deadzone,
    )
    from stm32_robot_bridge.serial_transport import (
        TransportStats,
        open_serial_port as create_serial_port,
        reset_serial_buffers as reset_device_buffers,
        set_modem_lines_low as lower_modem_lines,
        write_all,
    )
except ImportError:  # pragma: no cover - installed script fallback
    from bridge_state import BridgeState, BridgeStateMachine
    from command_guard import CommandGuard, CommandRejected
    from diagnostics import WarningThrottle
    from imu_converter import (
        ImuClockSynchronizer,
        InvalidImuSample,
        accel_g_to_mps2,
        gyro_dps_to_rad,
        normalize_quaternion,
    )
    from odometry import DifferentialOdometry, DynamicCovarianceModel, apply_deadzone
    from serial_transport import (
        TransportStats,
        open_serial_port as create_serial_port,
        reset_serial_buffers as reset_device_buffers,
        set_modem_lines_low as lower_modem_lines,
        write_all,
    )

try:
    from stm32_robot_bridge.protocol_v2 import (
        CMD_CLEAR_FAULT,
        CMD_DIAGNOSTIC,
        CMD_ESTOP,
        CMD_IMU_STATUS,
        CMD_LINE_CTRL,
        CMD_SET_VELOCITY,
        CMD_STATUS,
        DIAGNOSTIC_PAYLOAD_SIZE,
        FRAME_SOF,
        FRAME_SOF2,
        IMU_STATUS_PAYLOAD_SIZE,
        IMU_FLAG_CALIBRATED,
        IMU_FLAG_ERROR,
        IMU_FLAG_ONLINE,
        PROTOCOL_VERSION,
        STATUS_FLAG_ESTOP,
        STATUS_FLAG_FAULT_STOP,
        STATUS_PAYLOAD_SIZE,
        CommandStream,
        FrameParser,
        aggregate_status,
        build_frame,
        decode_diagnostic_payload,
        decode_imu_status_payload,
        decode_status_payload,
        encode_clear_fault_payload,
        encode_estop_payload,
        encode_line_ctrl_payload,
    )
except ImportError:  # pragma: no cover - installed script fallback
    from protocol_v2 import (
        CMD_CLEAR_FAULT,
        CMD_DIAGNOSTIC,
        CMD_ESTOP,
        CMD_IMU_STATUS,
        CMD_LINE_CTRL,
        CMD_SET_VELOCITY,
        CMD_STATUS,
        DIAGNOSTIC_PAYLOAD_SIZE,
        FRAME_SOF,
        FRAME_SOF2,
        IMU_STATUS_PAYLOAD_SIZE,
        IMU_FLAG_CALIBRATED,
        IMU_FLAG_ERROR,
        IMU_FLAG_ONLINE,
        PROTOCOL_VERSION,
        STATUS_FLAG_ESTOP,
        STATUS_FLAG_FAULT_STOP,
        STATUS_PAYLOAD_SIZE,
        CommandStream,
        FrameParser,
        aggregate_status,
        build_frame,
        decode_diagnostic_payload,
        decode_imu_status_payload,
        decode_status_payload,
        encode_clear_fault_payload,
        encode_estop_payload,
        encode_line_ctrl_payload,
    )


DEFAULT_BASE_PORT = "/dev/serial0"


class STM32Bridge(Node):
    def __init__(self) -> None:
        super().__init__("stm32_bridge")

        self.declare_parameter("port", DEFAULT_BASE_PORT)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("chassis_command_topic", "/chassis/command")
        self.declare_parameter("enable_legacy_cmd_vel", False)
        self.declare_parameter("legacy_cmd_vel_topic", "/cmd_vel/driver")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("imu.use_orientation", False)
        self.declare_parameter("imu.orientation_stddev", 0.2)
        self.declare_parameter("imu.angular_velocity_stddev.x", 0.02)
        self.declare_parameter("imu.angular_velocity_stddev.y", 0.02)
        self.declare_parameter("imu.angular_velocity_stddev.z", 0.02)
        self.declare_parameter("imu.linear_acceleration_stddev.x", 0.2)
        self.declare_parameter("imu.linear_acceleration_stddev.y", 0.2)
        self.declare_parameter("imu.linear_acceleration_stddev.z", 0.2)
        self.declare_parameter("imu.clock_offset_alpha", 0.02)
        self.declare_parameter("cmd_timeout", 0.15)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("status_hz", 100.0)
        self.declare_parameter("serial_open_retry_sec", 2.0)
        self.declare_parameter("status_timeout", 0.25)
        self.declare_parameter("drive_keepalive_sec", 0.05)
        self.declare_parameter("reject_non_finite", True)
        self.declare_parameter("hard_max_linear_mps", 0.45)
        self.declare_parameter("hard_max_angular_radps", 1.50)
        self.declare_parameter("max_command_age_sec", 0.15)
        self.declare_parameter("require_fresh_status_before_drive", True)
        self.declare_parameter("release_on_invalid_command", True)
        self.declare_parameter("release_on_status_timeout", True)
        self.declare_parameter("startup_settle_sec", 0.20)
        self.declare_parameter("wheel_radius", 0.0350)
        self.declare_parameter("wheel_track_width", 0.1760)
        self.declare_parameter("drive_mode", "differential")
        self.declare_parameter("odom_linear_scale", 1.0)
        self.declare_parameter("odom_angular_scale", 1.0)
        self.declare_parameter("odom_angular_sign", 1.0)
        self.declare_parameter("odom_linear_deadzone", 0.01)
        self.declare_parameter("odom_angular_deadzone", 0.03)
        self.declare_parameter("odom_max_dt_sec", 0.25)
        self.declare_parameter("status_log_interval_sec", 0.0)
        self.declare_parameter("cmd_log_interval_sec", 0.0)

        self.port = self._normalize_configured_port(str(self.get_parameter("port").value))
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.chassis_command_topic = str(self.get_parameter("chassis_command_topic").value)
        self.enable_legacy_cmd_vel = bool(self.get_parameter("enable_legacy_cmd_vel").value)
        self.legacy_cmd_vel_topic = str(self.get_parameter("legacy_cmd_vel_topic").value)
        self.odom_topic = self.get_parameter("odom_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self.imu_use_orientation = bool(self.get_parameter("imu.use_orientation").value)
        self.imu_orientation_stddev = float(self.get_parameter("imu.orientation_stddev").value)
        self.imu_angular_velocity_stddev = tuple(
            float(self.get_parameter(f"imu.angular_velocity_stddev.{axis}").value)
            for axis in ("x", "y", "z")
        )
        self.imu_linear_acceleration_stddev = tuple(
            float(self.get_parameter(f"imu.linear_acceleration_stddev.{axis}").value)
            for axis in ("x", "y", "z")
        )
        self.imu_clock_offset_alpha = float(self.get_parameter("imu.clock_offset_alpha").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.status_hz = float(self.get_parameter("status_hz").value)
        self.serial_open_retry_sec = float(self.get_parameter("serial_open_retry_sec").value)
        self.status_timeout = float(self.get_parameter("status_timeout").value)
        self.drive_keepalive_sec = float(self.get_parameter("drive_keepalive_sec").value)
        self.reject_non_finite = bool(self.get_parameter("reject_non_finite").value)
        self.hard_max_linear_mps = float(self.get_parameter("hard_max_linear_mps").value)
        self.hard_max_angular_radps = float(self.get_parameter("hard_max_angular_radps").value)
        self.max_command_age_sec = float(self.get_parameter("max_command_age_sec").value)
        self.require_fresh_status_before_drive = bool(
            self.get_parameter("require_fresh_status_before_drive").value
        )
        self.release_on_invalid_command = bool(self.get_parameter("release_on_invalid_command").value)
        self.release_on_status_timeout = bool(self.get_parameter("release_on_status_timeout").value)
        self.startup_settle_sec = float(self.get_parameter("startup_settle_sec").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_track_width = float(self.get_parameter("wheel_track_width").value)
        self.drive_mode = str(self.get_parameter("drive_mode").value)
        self.odom_linear_scale = float(self.get_parameter("odom_linear_scale").value)
        self.odom_angular_scale = float(self.get_parameter("odom_angular_scale").value)
        self.odom_angular_sign = 1.0 if float(self.get_parameter("odom_angular_sign").value) >= 0.0 else -1.0
        self.odom_linear_deadzone = float(self.get_parameter("odom_linear_deadzone").value)
        self.odom_angular_deadzone = float(self.get_parameter("odom_angular_deadzone").value)
        self.odom_max_dt_sec = float(self.get_parameter("odom_max_dt_sec").value)
        self.status_log_interval_sec = float(self.get_parameter("status_log_interval_sec").value)
        self.cmd_log_interval_sec = float(self.get_parameter("cmd_log_interval_sec").value)

        self.serial = None
        self.connected_port = ""
        self.parser = FrameParser()
        self.transport_stats = TransportStats()
        self.stats_started_monotonic = time.monotonic()
        self.startup_release_remaining = 0
        self.next_startup_action_sec = None
        self.last_open_attempt = 0.0
        self.last_missing_port_log = 0.0
        self.last_cmd_log_time = 0.0
        self.last_drive_log_time = 0.0
        self.last_status_log_time = 0.0
        self.warning_throttle = WarningThrottle()
        self.warn_times: Dict[str, float] = self.warning_throttle.last_emit

        self.command_stream = CommandStream(self.cmd_timeout, self.drive_keepalive_sec)
        self.command_guard = CommandGuard(
            hard_max_linear_mps=self.hard_max_linear_mps,
            hard_max_angular_radps=self.hard_max_angular_radps,
            max_command_age_sec=self.max_command_age_sec,
            status_timeout_sec=self.status_timeout,
        )
        self.state_machine = BridgeStateMachine()
        self.has_seen_cmd_vel = False
        self.command_sequence = 0
        self.selected_source = ChassisCommand.SOURCE_NONE
        self.invalid_command_count = 0
        self.release_count = 0
        self.target_vx = 0.0
        self.target_wz = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odometry = DifferentialOdometry(max_dt_sec=self.odom_max_dt_sec)
        self.odom_covariance_model = DynamicCovarianceModel()
        self.last_status_ts = None
        self.last_status_monotonic = None

        self.protocol_version = 0
        self.status_flags = 0
        self.control_source = 0
        self.motor_enabled_mask = 0
        self.motor_speed_valid_mask = 0
        self.encoder_anomaly_mask = 0
        self.comm_health_flags = 0
        self.feedback_error_flags = 0
        self.feedback_latched_error_flags = 0
        self.feedback_battery_voltage = 0.0
        self.feedback_left_current = 0.0
        self.feedback_right_current = 0.0
        self.feedback_left_speed = 0.0
        self.feedback_right_speed = 0.0
        self.feedback_left_target = 0.0
        self.feedback_right_target = 0.0
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.feedback_vx = 0.0
        self.feedback_wz = 0.0
        self.feedback_odom_trusted = False
        self.imu_online = False
        self.last_imu_monotonic = None
        self.imu_quality_flags = 0
        self.imu_status_flags = 0
        self.imu_sample_count = 0
        self.imu_temperature_c = 0
        self.invalid_imu_count = 0
        self.imu_dropped_samples = 0
        self.imu_clock = ImuClockSynchronizer(self.imu_clock_offset_alpha)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None
        self.chassis_status_pub = self.create_publisher(UInt32, "/chassis/status", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.left_current_pub = self.create_publisher(Float32, "/motor/left_current", 10)
        self.right_current_pub = self.create_publisher(Float32, "/motor/right_current", 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 20)
        self.diag_pub = self.create_publisher(UInt32, "/chassis/diagnostic", 10)
        self.state_pub = self.create_publisher(ChassisState, "/chassis/state", 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_subscription(ChassisCommand, self.chassis_command_topic, self.on_chassis_command, 20)
        if self.enable_legacy_cmd_vel:
            self.create_subscription(Twist, self.legacy_cmd_vel_topic, self.on_cmd_vel, 20)
        self.create_service(SetBool, "/chassis/estop", self.on_estop)
        self.create_service(Trigger, "/chassis/clear_fault", self.on_clear_fault)
        self.create_service(SetBool, "/chassis/line_ctrl", self.on_line_ctrl)

        self.create_timer(1.0 / max(self.control_hz, 1.0), self.control_loop)
        self.create_timer(1.0 / max(self.status_hz, 1.0), self.status_poll)
        self.create_timer(1.0, self.publish_diagnostics)

        self.ensure_serial(force_log_missing=True)
        self.get_logger().info(
            "STM32 bridge v2 started: "
            f"port={self.port} baud={self.baudrate} protocol={PROTOCOL_VERSION} "
            f"wheel_track_width={self.wheel_track_width:.4f} wheel_radius={self.wheel_radius:.4f} "
            f"drive_mode={self.drive_mode} "
            f"publish_tf={self.publish_tf}"
        )

    def _normalize_configured_port(self, requested: str) -> str:
        requested = requested.strip()
        env_hint = os.environ.get("ROBOT_BASE_PORT_HINT", "").strip()
        if requested.lower() == "auto":
            self.get_logger().warn("base port 'auto' is deprecated for GPIO UART; using /dev/serial0")
            return env_hint or DEFAULT_BASE_PORT
        return requested or env_hint or DEFAULT_BASE_PORT

    def ensure_serial(self, force_log_missing: bool = False) -> bool:
        if self.serial is not None:
            return True
        if serial is None:
            self.get_logger().error("python3-serial not installed")
            return False

        now_monotonic = time.monotonic()
        if not force_log_missing and (now_monotonic - self.last_open_attempt) < self.serial_open_retry_sec:
            return False
        self.last_open_attempt = now_monotonic

        if not os.path.exists(self.port):
            if force_log_missing or (now_monotonic - self.last_missing_port_log) >= 5.0:
                self.get_logger().warn(f"Serial port {self.port} not available yet; bridge will keep retrying")
                self.last_missing_port_log = now_monotonic
            return False

        try:
            self.serial = self.open_serial_port(self.port)
            self.connected_port = self.port
            self.parser = FrameParser()
            self.command_stream = CommandStream(self.cmd_timeout, self.drive_keepalive_sec)
            self.state_machine.on_serial_opened()
            self.transport_stats.serial_reconnects += 1
            self.startup_release_remaining = 3
            self.next_startup_action_sec = now_monotonic + max(self.startup_settle_sec, 0.0)
            self.get_logger().info(f"Opened STM32 serial {self.port}@{self.baudrate}")
            self.set_modem_lines_low()
            self.reset_serial_buffers()
            return True
        except Exception as exc:
            self.serial = None
            self.connected_port = ""
            self.startup_release_remaining = 0
            self.next_startup_action_sec = None
            self.state_machine.on_disconnected()
            self.get_logger().warn(f"Failed to open serial {self.port}: {exc}")
            return False

    def open_serial_port(self, resolved_port: str):
        return create_serial_port(serial, resolved_port, self.baudrate)

    def set_modem_lines_low(self) -> None:
        lower_modem_lines(self.serial)

    def reset_serial_buffers(self) -> None:
        reset_device_buffers(self.serial)

    def _advance_startup(self, now_monotonic: float) -> None:
        if (
            self.serial is None
            or self.state_machine.state is not BridgeState.SETTLING
            or self.next_startup_action_sec is None
            or now_monotonic < self.next_startup_action_sec
        ):
            return
        if not self.release_upper_control():
            return
        self.startup_release_remaining -= 1
        if self.startup_release_remaining > 0:
            self.next_startup_action_sec = now_monotonic + 0.02
            return
        self.next_startup_action_sec = None
        self.state_machine.on_settled()

    def release_upper_control(self) -> bool:
        if self.serial is None:
            return False
        released = self.command_stream.release(
            lambda payload: self.write_frame(CMD_SET_VELOCITY, payload)
        )
        if released:
            self.release_count += 1
        return released

    def close_serial(self, reason: str, release_control: bool = True) -> None:
        port = self.connected_port
        if self.serial is not None and release_control:
            self.release_upper_control()
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.connected_port = ""
        self.parser = FrameParser()
        self.startup_release_remaining = 0
        self.next_startup_action_sec = None
        self.odometry.reset_sample_baseline()
        self.imu_clock.reset()
        self.state_machine.on_disconnected()
        if reason:
            self.get_logger().warn(reason if not port else f"{reason} ({port})")

    def write_frame(self, cmd: int, payload: bytes) -> bool:
        if self.serial is None:
            return False
        frame = build_frame(cmd, payload)
        try:
            write_all(self.serial, frame, self.transport_stats)
            self.transport_stats.tx_frames += 1
            self.transport_stats.tx_bytes += len(frame)
            return True
        except Exception as exc:
            self.transport_stats.tx_errors += 1
            self.close_serial(f"Serial write failed: {exc}", release_control=False)
            return False

    def on_estop(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Handle /chassis/estop service.

        NOTE: The STM32 firmware intentionally ignores estop=0 from remote
        sources (USART3/ESP12F). ESTOP release is local-only via USART1
        debug console. See firmware docs/upper-protocol-v2.md.
        Reject estop=0 locally so callers cannot mistake a transmitted but
        ignored frame for a successful release.
        """
        enabled = bool(request.data)
        if not enabled:
            response.success = False
            response.message = (
                "remote estop release is unsupported; "
                "use local USART1 debug console 'estop 0'"
            )
            self.get_logger().warn("Rejected remote ESTOP release; use local USART1 console: 'estop 0'")
            return response

        if not self.ensure_serial():
            response.success = False
            response.message = "serial not connected"
            return response
        sent = self.write_frame(CMD_ESTOP, encode_estop_payload(True))
        response.success = sent
        response.message = "estop triggered" if sent else "send failed"
        if sent:
            self.get_logger().warn("Sent ESTOP=1 to STM32")
        return response

    def on_cmd_vel(self, msg: Twist) -> None:
        if not self.enable_legacy_cmd_vel:
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._accept_drive_command(
            vx=msg.linear.x,
            wz=msg.angular.z,
            command_stamp_sec=now_sec,
            source=ChassisCommand.SOURCE_NONE,
        )

    def on_chassis_command(self, msg: ChassisCommand) -> None:
        self.command_sequence = int(msg.sequence) & 0xFFFFFFFF
        if not bool(msg.enable):
            self.selected_source = ChassisCommand.SOURCE_NONE
            self.target_vx = 0.0
            self.target_wz = 0.0
            self.release_upper_control()
            self.command_stream.clear_command()
            self.state_machine.on_disable_command()
            return

        try:
            command_stamp_sec = self._stamp_to_seconds(msg.header.stamp)
        except CommandRejected as exc:
            self._reject_drive_command(str(exc))
            return
        self._accept_drive_command(
            vx=msg.twist.linear.x,
            wz=msg.twist.angular.z,
            command_stamp_sec=command_stamp_sec,
            source=int(msg.source) & 0xFF,
        )

    @property
    def bridge_state(self) -> BridgeState:
        return self.state_machine.state

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        if stamp is None or not hasattr(stamp, "sec") or not hasattr(stamp, "nanosec"):
            raise CommandRejected("command timestamp is missing")
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _status_age_sec(self, now_monotonic: float) -> Optional[float]:
        if self.last_status_monotonic is None:
            return None
        return now_monotonic - self.last_status_monotonic

    def _accept_drive_command(self, *, vx, wz, command_stamp_sec: float, source: int) -> bool:
        now_ros_sec = self.get_clock().now().nanoseconds * 1e-9
        now_monotonic = time.monotonic()
        try:
            guarded = self.command_guard.validate(
                vx=vx,
                wz=wz,
                command_stamp_sec=command_stamp_sec,
                now_sec=now_ros_sec,
                status_age_sec=self._status_age_sec(now_monotonic),
                drive_permitted=self.serial is not None and self.state_machine.can_drive,
                status_flags=self.status_flags,
                protocol_version=self.protocol_version,
            )
        except CommandRejected as exc:
            self._reject_drive_command(str(exc))
            return False

        self.has_seen_cmd_vel = True
        self.selected_source = source
        self.target_vx = guarded.vx
        self.target_wz = guarded.wz
        self.command_stream.update_command(self.target_vx, self.target_wz, now_monotonic)
        self.state_machine.on_drive_enabled()
        self.log_cmd_vel_rx(self.target_vx, self.target_wz)
        return True

    def _reject_drive_command(self, reason: str) -> None:
        self.invalid_command_count += 1
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.selected_source = ChassisCommand.SOURCE_NONE
        if self.release_on_invalid_command:
            self.release_upper_control()
        self.command_stream.clear_command()
        self.warn_periodic("invalid_command", f"Rejected chassis command: {reason}")

    def control_loop(self) -> None:
        self.ensure_serial()
        self.read_serial_frames()

        now_sec = time.monotonic()
        self._advance_startup(now_sec)
        self._enforce_status_freshness(now_sec)
        if self.state_machine.can_drive:
            self.command_stream.tick(
                now_sec, lambda payload: self.write_drive_payload(payload, now_sec)
            )
            if (
                self.state_machine.state is BridgeState.ACTIVE
                and self.command_stream.last_command_time is not None
                and (now_sec - self.command_stream.last_command_time) > self.cmd_timeout
            ):
                self.command_stream.clear_command()
                self.state_machine.on_disable_command()

    def _enforce_status_freshness(self, now_monotonic: float) -> None:
        age = self._status_age_sec(now_monotonic)
        if age is None or age <= self.status_timeout:
            return
        was_motion_capable = self.state_machine.state in (
            BridgeState.READY,
            BridgeState.ACTIVE,
            BridgeState.FAULT,
        )
        if was_motion_capable and self.release_on_status_timeout:
            self.release_upper_control()
        self.command_stream.clear_command()
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.selected_source = ChassisCommand.SOURCE_NONE
        self.state_machine.on_status_timeout()

    def write_drive_payload(self, payload: bytes, now_sec: float) -> bool:
        sent = self.write_frame(CMD_SET_VELOCITY, payload)
        enable = payload[8] if len(payload) >= 9 else 0
        vx, wz = self.target_vx, self.target_wz
        if enable == 0:
            vx, wz = 0.0, 0.0
        self.log_drive_tx(vx, wz, sent, enable != 0, now_sec)
        return sent

    def status_poll(self) -> None:
        if not self.ensure_serial():
            return
        self._advance_startup(time.monotonic())
        self.read_serial_frames()

    def read_serial_frames(self) -> None:
        if self.serial is None:
            return
        try:
            waiting = self.serial.in_waiting
            if waiting <= 0:
                return
            chunk = self.serial.read(waiting)
            self.transport_stats.rx_bytes += len(chunk)
        except Exception as exc:
            self.close_serial(f"Serial read failed: {exc}", release_control=False)
            return

        frames = self.parser.feed(chunk)
        self.transport_stats.rx_crc_errors = self.parser.stats.crc_errors
        self.transport_stats.rx_bad_length = self.parser.stats.bad_length
        self.transport_stats.rx_resync_bytes = self.parser.stats.resync_bytes
        self.transport_stats.rx_frames += len(frames)
        for cmd, payload in frames:
            self.handle_frame(cmd, payload)

    def handle_frame(self, cmd: int, payload: bytes) -> None:
        if cmd == CMD_STATUS:
            self.handle_status_frame(payload)
            return
        if cmd == CMD_IMU_STATUS:
            self.handle_imu_status_frame(payload)
            return
        if cmd == CMD_DIAGNOSTIC:
            self.handle_diagnostic_frame(payload)
            return
        self.transport_stats.rx_unknown_cmd += 1
        self.get_logger().debug(f"Unknown frame cmd=0x{cmd:02X} len={len(payload)}")

    def handle_status_frame(self, payload: bytes) -> None:
        if len(payload) != STATUS_PAYLOAD_SIZE:
            self.transport_stats.rx_bad_length += 1
            self.warn_periodic("bad_status_len", f"Discard STATUS payload len={len(payload)} expected={STATUS_PAYLOAD_SIZE}")
            return

        status = decode_status_payload(payload)
        if status is None:
            self.transport_stats.rx_bad_version += 1
            version = payload[0] if payload else -1
            self.warn_periodic("bad_status_version", f"Discard STATUS protocol version={version} expected={PROTOCOL_VERSION}")
            return

        agg = aggregate_status(status, self.wheel_track_width, drive_mode=self.drive_mode)
        self.last_status_ts = self.get_clock().now()
        self.last_status_monotonic = time.monotonic()
        self.protocol_version = status.version
        self.status_flags = status.status_flags
        self.control_source = status.control_source
        self.motor_enabled_mask = status.motor_enabled_mask
        self.motor_speed_valid_mask = status.motor_speed_valid_mask
        self.encoder_anomaly_mask = status.encoder_anomaly_mask
        self.comm_health_flags = status.comm_health_flags
        self.feedback_error_flags = status.error_flags
        self.feedback_latched_error_flags = status.latched_error_flags
        self.feedback_battery_voltage = status.battery_voltage
        self.feedback_left_current = agg.left_current_a
        self.feedback_right_current = agg.right_current_a
        self.feedback_left_speed = agg.left_speed_mps
        self.feedback_right_speed = agg.right_speed_mps
        self.feedback_left_target = agg.left_target_mps
        self.feedback_right_target = agg.right_target_mps
        self.last_left_encoder = agg.left_encoder_count
        self.last_right_encoder = agg.right_encoder_count
        self.feedback_odom_trusted = agg.odom_trusted

        self.feedback_vx = agg.vx_mps * self.odom_linear_scale
        self.feedback_wz = agg.wz_radps * self.odom_angular_scale * self.odom_angular_sign

        previous_state = self.state_machine.state
        self.state_machine.on_valid_status(status.status_flags)
        if (
            self.state_machine.state is BridgeState.FAULT
            and previous_state is not BridgeState.FAULT
        ):
            self.release_upper_control()
            self.command_stream.clear_command()
            self.target_vx = 0.0
            self.target_wz = 0.0
            self.selected_source = ChassisCommand.SOURCE_NONE

        self.update_and_publish_odom(self.last_status_ts, self.last_status_monotonic)

        self.publish_auxiliary_topics()
        self.log_status_summary()

    def handle_imu_status_frame(self, payload: bytes) -> None:
        if len(payload) != IMU_STATUS_PAYLOAD_SIZE:
            self.transport_stats.rx_bad_length += 1
            self.warn_periodic("bad_imu_status_len", f"Discard IMU_STATUS payload len={len(payload)} expected={IMU_STATUS_PAYLOAD_SIZE}")
            return

        imu = decode_imu_status_payload(payload)
        if imu is None:
            self.transport_stats.rx_bad_version += 1
            version = payload[0] if payload else -1
            self.warn_periodic("bad_imu_status_version", f"Discard IMU_STATUS protocol version={version} expected={PROTOCOL_VERSION}")
            return

        self.imu_online = bool(imu.status_flags & IMU_FLAG_ONLINE)
        self.imu_quality_flags = imu.quality_flags
        self.imu_status_flags = imu.status_flags
        self.imu_sample_count = imu.sample_count
        self.imu_temperature_c = imu.temperature_c

        q = normalize_quaternion(imu.quaternion)
        vectors_finite = all(
            math.isfinite(float(value))
            for values in (imu.gyro_corrected_dps, imu.accel_g)
            for value in values
        )
        if (
            not self.imu_online
            or bool(imu.status_flags & IMU_FLAG_ERROR)
            or imu.quality_flags != 0
            or q is None
            or not vectors_finite
        ):
            self.invalid_imu_count += 1
            self.warn_periodic(
                "invalid_imu",
                "Rejected IMU_STATUS sample due to status, quality, quaternion, or vector validation",
            )
            return

        receive_time = self.get_clock().now()
        receive_ros_sec = receive_time.nanoseconds * 1e-9
        try:
            timing = self.imu_clock.update(
                timestamp_ms=imu.timestamp_ms,
                sample_count=imu.sample_count,
                receive_ros_sec=receive_ros_sec,
            )
        except InvalidImuSample as exc:
            self.invalid_imu_count += 1
            self.warn_periodic("invalid_imu_time", f"Rejected IMU_STATUS timing: {exc}")
            return
        if timing.reset:
            self.odometry.reset_sample_baseline()
        self.imu_dropped_samples = self.imu_clock.total_dropped_samples
        self.last_imu_monotonic = time.monotonic()

        msg = Imu()
        msg.header.stamp = self._seconds_to_time_message(timing.sample_ros_sec)
        msg.header.frame_id = self.imu_frame_id

        if not self.imu_use_orientation:
            msg.orientation_covariance[0] = -1.0
        else:
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            variance = self.imu_orientation_stddev ** 2
            msg.orientation_covariance[0] = variance
            msg.orientation_covariance[4] = variance
            msg.orientation_covariance[8] = variance

        gyro = gyro_dps_to_rad(imu.gyro_corrected_dps)
        accel = accel_g_to_mps2(imu.accel_g)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        for index, stddev in zip((0, 4, 8), self.imu_angular_velocity_stddev):
            msg.angular_velocity_covariance[index] = stddev ** 2
        for index, stddev in zip((0, 4, 8), self.imu_linear_acceleration_stddev):
            msg.linear_acceleration_covariance[index] = stddev ** 2

        self.imu_pub.publish(msg)

    @staticmethod
    def _seconds_to_time_message(seconds: float) -> TimeMessage:
        sec = math.floor(seconds)
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return TimeMessage(sec=int(sec), nanosec=nanosec)

    def handle_diagnostic_frame(self, payload: bytes) -> None:
        if len(payload) != DIAGNOSTIC_PAYLOAD_SIZE:
            self.transport_stats.rx_bad_length += 1
            self.warn_periodic("bad_diag_len", f"Discard DIAGNOSTIC payload len={len(payload)} expected={DIAGNOSTIC_PAYLOAD_SIZE}")
            return

        diag = decode_diagnostic_payload(payload)
        if diag is None:
            self.transport_stats.rx_bad_version += 1
            version = payload[0] if payload else -1
            self.warn_periodic("bad_diag_version", f"Discard DIAGNOSTIC protocol version={version} expected={PROTOCOL_VERSION}")
            return

        # Publish diagnostic summary on /chassis/diagnostic topic
        self.diag_pub.publish(UInt32(data=diag.post_error_flags))

        # Log noteworthy diagnostic events
        if diag.post_done and diag.post_error_flags != 0:
            self.warn_periodic("post_errors", f"STM32 POST errors: 0x{diag.post_error_flags:08X}")
        if diag.adc_invalid_reason_flags != 0:
            self.warn_periodic("adc_invalid", f"STM32 ADC invalid: 0x{diag.adc_invalid_reason_flags:08X}")
        if diag.task_timeout_mask != 0:
            self.warn_periodic("task_timeout", f"STM32 task timeout mask: 0x{diag.task_timeout_mask:04X}")
        self.get_logger().debug(
            f"DIAGNOSTIC: uptime={diag.uptime_ms}ms reset_reason=0x{diag.reset_reason_flags:08X} "
            f"imu_quality=0x{diag.imu_quality_flags:08X}"
        )

    def on_clear_fault(self, request, response):
        """Handle /chassis/clear_fault service (std_srvs/Trigger).

        Sends CLEAR_FAULT (0x04) with zero-length payload.
        NOTE: Cannot clear ESTOP; that requires local USART1 console.
        """
        if not self.ensure_serial():
            response.success = False
            response.message = "serial not connected"
            return response
        sent = self.write_frame(CMD_CLEAR_FAULT, encode_clear_fault_payload())
        response.success = sent
        response.message = "clear_fault sent" if sent else "send failed"
        if sent:
            self.get_logger().info("Sent CLEAR_FAULT to STM32")
        return response

    def on_line_ctrl(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Handle /chassis/line_ctrl service (std_srvs/SetBool)."""
        if not self.ensure_serial():
            response.success = False
            response.message = "serial not connected"
            return response
        enabled = bool(request.data)
        sent = self.write_frame(CMD_LINE_CTRL, encode_line_ctrl_payload(enabled))
        response.success = sent
        response.message = f"line_ctrl {'enabled' if enabled else 'disabled'}" if sent else "send failed"
        if sent:
            self.get_logger().info(f"Sent LINE_CTRL={'ON' if enabled else 'OFF'} to STM32")
        return response

    def publish_auxiliary_topics(self) -> None:
        if self.last_status_ts is None:
            return
        self.chassis_status_pub.publish(UInt32(data=self.feedback_error_flags))

        batt = BatteryState()
        batt.header.stamp = self.last_status_ts.to_msg()
        batt.voltage = self.feedback_battery_voltage
        batt.present = self.feedback_battery_voltage > 1.0
        self.battery_pub.publish(batt)

        self.left_current_pub.publish(Float32(data=self.feedback_left_current))
        self.right_current_pub.publish(Float32(data=self.feedback_right_current))
        self.publish_chassis_state()

    def publish_chassis_state(self) -> None:
        state = ChassisState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.bridge_state = int(self.state_machine.state)
        state.selected_source = int(self.selected_source)
        state.command_sequence = int(self.command_sequence)
        state.upper_enabled = self.state_machine.state is BridgeState.ACTIVE
        state.firmware_control_source = int(self.control_source)
        state.status_flags = int(self.status_flags)
        state.error_flags = int(self.feedback_error_flags)
        state.latched_error_flags = int(self.feedback_latched_error_flags)
        self.state_pub.publish(state)

    def publish_diagnostics(self) -> None:
        now_monotonic = time.monotonic()
        elapsed = max(now_monotonic - self.stats_started_monotonic, 1e-6)
        status_age = self._status_age_sec(now_monotonic)
        command_age = (
            None
            if self.command_stream.last_command_time is None
            else now_monotonic - self.command_stream.last_command_time
        )
        connected = self.serial is not None
        serial_level = DiagnosticStatus.OK if connected else DiagnosticStatus.ERROR
        serial_message = "connected" if connected else "disconnected"
        if connected and self.state_machine.state in (BridgeState.SETTLING, BridgeState.WAIT_STATUS):
            serial_level = DiagnosticStatus.WARN
            serial_message = self.state_machine.state.name.lower()

        protocol_level = DiagnosticStatus.OK
        protocol_message = "upper v2 healthy"
        if self.protocol_version != PROTOCOL_VERSION:
            protocol_level = DiagnosticStatus.ERROR
            protocol_message = "protocol v2 STATUS unavailable"
        elif self.transport_stats.rx_crc_errors or self.transport_stats.rx_bad_length:
            protocol_level = DiagnosticStatus.WARN
            protocol_message = "parser errors observed"

        control_level = DiagnosticStatus.OK
        control_message = self.state_machine.state.name.lower()
        if self.state_machine.state is BridgeState.FAULT:
            control_level = DiagnosticStatus.ERROR
        elif not self.state_machine.can_drive:
            control_level = DiagnosticStatus.WARN

        chassis_fault = bool(self.status_flags & (STATUS_FLAG_ESTOP | STATUS_FLAG_FAULT_STOP))
        chassis_level = DiagnosticStatus.ERROR if chassis_fault else DiagnosticStatus.OK
        chassis_message = "ESTOP/fault-stop" if chassis_fault else "healthy"

        imu_age = None if self.last_imu_monotonic is None else now_monotonic - self.last_imu_monotonic
        if not self.imu_online or imu_age is None or imu_age > self.status_timeout:
            imu_level = DiagnosticStatus.ERROR
            imu_message = "IMU_STATUS stale or unavailable"
        elif self.imu_status_flags & IMU_FLAG_ERROR:
            imu_level = DiagnosticStatus.ERROR
            imu_message = "IMU error flag set"
        elif self.imu_quality_flags:
            imu_level = DiagnosticStatus.WARN
            imu_message = "IMU quality flags set"
        else:
            imu_level = DiagnosticStatus.OK
            imu_message = "healthy"

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [
            self._make_diagnostic(
                "stm32_bridge/serial",
                serial_level,
                serial_message,
                {
                    "connected": connected,
                    "port": self.connected_port or self.port,
                    "baudrate": self.baudrate,
                    "reconnect_count": self.transport_stats.serial_reconnects,
                    "rx_hz": f"{self.transport_stats.rx_frames / elapsed:.2f}",
                    "tx_hz": f"{self.transport_stats.tx_frames / elapsed:.2f}",
                    "last_rx_age": "none" if status_age is None else f"{status_age:.3f}",
                    "short_write_count": self.transport_stats.tx_short_writes,
                },
            ),
            self._make_diagnostic(
                "stm32_bridge/protocol",
                protocol_level,
                protocol_message,
                {
                    "protocol_version": self.protocol_version,
                    "crc_error_count": self.transport_stats.rx_crc_errors,
                    "bad_length_count": self.transport_stats.rx_bad_length,
                    "bad_version_count": self.transport_stats.rx_bad_version,
                    "unknown_command_count": self.transport_stats.rx_unknown_cmd,
                    "parser_resync_count": self.transport_stats.rx_resync_bytes,
                },
            ),
            self._make_diagnostic(
                "stm32_bridge/control",
                control_level,
                control_message,
                {
                    "bridge_state": self.state_machine.state.name,
                    "selected_source": self.selected_source,
                    "command_sequence": self.command_sequence,
                    "command_age": "none" if command_age is None else f"{command_age:.3f}",
                    "upper_enabled": self.state_machine.state is BridgeState.ACTIVE,
                    "firmware_control_source": self.control_source,
                    "release_count": self.release_count,
                    "invalid_command_count": self.invalid_command_count,
                },
            ),
            self._make_diagnostic(
                "stm32_bridge/chassis",
                chassis_level,
                chassis_message,
                {
                    "estop": bool(self.status_flags & STATUS_FLAG_ESTOP),
                    "fault_stop": bool(self.status_flags & STATUS_FLAG_FAULT_STOP),
                    "error_flags": f"0x{self.feedback_error_flags:08X}",
                    "latched_error_flags": f"0x{self.feedback_latched_error_flags:08X}",
                    "motor_enabled_mask": f"0x{self.motor_enabled_mask:02X}",
                    "encoder_anomaly_mask": f"0x{self.encoder_anomaly_mask:02X}",
                    "battery_voltage": f"{self.feedback_battery_voltage:.3f}",
                },
            ),
            self._make_diagnostic(
                "stm32_bridge/imu",
                imu_level,
                imu_message,
                {
                    "online": bool(self.imu_status_flags & IMU_FLAG_ONLINE),
                    "calibrated": bool(self.imu_status_flags & IMU_FLAG_CALIBRATED),
                    "quality_flags": f"0x{self.imu_quality_flags:08X}",
                    "sample_count": self.imu_sample_count,
                    "dropped_samples": self.imu_dropped_samples,
                    "invalid_samples": self.invalid_imu_count,
                    "temperature": self.imu_temperature_c,
                    "timestamp_age": "none" if imu_age is None else f"{imu_age:.3f}",
                },
            ),
        ]
        self.diagnostics_pub.publish(array)

    def _make_diagnostic(self, name: str, level: int, message: str, values) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = name
        status.level = level
        status.message = message
        status.hardware_id = self.connected_port or "stm32_chassis"
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        return status

    def update_and_publish_odom(self, now, sample_monotonic: float) -> None:
        status_allows_motion = (self.status_flags & (STATUS_FLAG_ESTOP | STATUS_FLAG_FAULT_STOP)) == 0
        sample_trusted = self.feedback_odom_trusted and status_allows_motion
        sample_vx = self.apply_deadzone(
            self.feedback_vx if sample_trusted else 0.0,
            self.odom_linear_deadzone,
        )
        sample_wz = self.apply_deadzone(
            self.feedback_wz if sample_trusted else 0.0,
            self.odom_angular_deadzone,
        )
        update = self.odometry.update_from_wheel_speeds(
            sample_vx,
            sample_wz,
            sample_time_sec=sample_monotonic,
            trusted=sample_trusted,
        )
        self.x, self.y, self.yaw = update.pose.x, update.pose.y, update.pose.yaw
        odom_vx, odom_wz = update.vx, update.wz
        odom_trusted = update.trusted
        covariance = self.odom_covariance_model.estimate(
            vx=odom_vx,
            wz=odom_wz,
            dt=max(update.dt, 0.0),
            wheel_disagreement=abs(self.feedback_right_speed - self.feedback_left_speed),
            trusted=odom_trusted,
        )

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = odom_vx
        odom.twist.twist.angular.z = odom_wz

        odom.pose.covariance[0] = covariance.pose_xy_variance
        odom.pose.covariance[7] = covariance.pose_xy_variance
        odom.pose.covariance[35] = covariance.pose_yaw_variance
        odom.twist.covariance[0] = covariance.linear_variance
        odom.twist.covariance[35] = covariance.angular_variance

        self.odom_pub.publish(odom)

        if self.tf_br is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = self.child_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_br.sendTransform(tf_msg)

    @staticmethod
    def apply_deadzone(value: float, threshold: float) -> float:
        return apply_deadzone(value, threshold)

    def warn_periodic(self, key: str, message: str, interval_sec: float = 2.0) -> None:
        now_monotonic = time.monotonic()
        if self.warning_throttle.should_emit(key, now_monotonic, interval_sec):
            self.get_logger().warn(message)

    def log_cmd_vel_rx(self, vx: float, wz: float) -> None:
        if self.cmd_log_interval_sec <= 0.0:
            return
        now_monotonic = time.monotonic()
        if (now_monotonic - self.last_cmd_log_time) < self.cmd_log_interval_sec:
            return
        self.last_cmd_log_time = now_monotonic
        status_age = "none"
        if self.last_status_ts is not None:
            age = (self.get_clock().now() - self.last_status_ts).nanoseconds * 1e-9
            status_age = f"{age:.2f}s"
        self.get_logger().info(
            "cmd_vel_rx "
            f"cmd=({vx:.3f}m/s,{math.degrees(wz):.2f}deg/s) "
            f"serial={self.connected_port or 'none'} "
            f"control_source={self.control_source} status_age={status_age}"
        )

    def log_drive_tx(self, vx: float, wz: float, sent: bool, active: bool, now_monotonic: float) -> None:
        if self.cmd_log_interval_sec <= 0.0:
            return
        if (now_monotonic - self.last_drive_log_time) < self.cmd_log_interval_sec:
            return
        self.last_drive_log_time = now_monotonic
        self.get_logger().info(
            "drive_tx "
            f"sent={1 if sent else 0} active={1 if active else 0} "
            f"cmd=({vx:.3f}m/s,{math.degrees(wz):.2f}deg/s) "
            f"serial={self.connected_port or 'none'} "
            f"control_source={self.control_source}"
        )

    def log_status_summary(self) -> None:
        if self.status_log_interval_sec <= 0.0:
            return
        now_monotonic = time.monotonic()
        if (now_monotonic - self.last_status_log_time) < self.status_log_interval_sec:
            return
        self.last_status_log_time = now_monotonic
        self.get_logger().info(
            "status_summary "
            f"protocol={self.protocol_version} "
            f"control_source={self.control_source} "
            f"status_flags=0x{self.status_flags:02X} "
            f"enabled_mask=0x{self.motor_enabled_mask:02X} "
            f"speed_valid_mask=0x{self.motor_speed_valid_mask:02X} "
            f"odom_trusted={1 if self.feedback_odom_trusted else 0} "
            f"odom=({self.feedback_vx:.3f}m/s,{math.degrees(self.feedback_wz):.2f}deg/s) "
            f"wheels=({self.feedback_left_speed:.3f},{self.feedback_right_speed:.3f})m/s "
            f"targets=({self.feedback_left_target:.3f},{self.feedback_right_target:.3f})m/s "
            f"enc=({self.last_left_encoder},{self.last_right_encoder}) "
            f"bat={self.feedback_battery_voltage:.2f}V "
            f"cur=({self.feedback_left_current:.2f},{self.feedback_right_current:.2f})A "
            f"err=0x{self.feedback_error_flags:08X} "
            f"latched=0x{self.feedback_latched_error_flags:08X}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = STM32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial("STM32 bridge shutting down", release_control=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
