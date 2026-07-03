#!/usr/bin/env python3
import math
import os
import time
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, UInt32
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

try:
    import serial
except Exception:  # pragma: no cover
    serial = None

try:
    from stm32_robot_bridge.protocol_v2 import (
        CMD_ESTOP,
        CMD_SET_VELOCITY,
        CMD_STATUS,
        FRAME_SOF,
        FRAME_SOF2,
        PROTOCOL_VERSION,
        STATUS_FLAG_ESTOP,
        STATUS_FLAG_FAULT_STOP,
        STATUS_PAYLOAD_SIZE,
        CommandStream,
        FrameParser,
        aggregate_status,
        build_frame,
        decode_status_payload,
        encode_estop_payload,
    )
except ImportError:  # pragma: no cover - installed script fallback
    from protocol_v2 import (
        CMD_ESTOP,
        CMD_SET_VELOCITY,
        CMD_STATUS,
        FRAME_SOF,
        FRAME_SOF2,
        PROTOCOL_VERSION,
        STATUS_FLAG_ESTOP,
        STATUS_FLAG_FAULT_STOP,
        STATUS_PAYLOAD_SIZE,
        CommandStream,
        FrameParser,
        aggregate_status,
        build_frame,
        decode_status_payload,
        encode_estop_payload,
    )


DEFAULT_BASE_PORT = "/dev/serial0"


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class STM32Bridge(Node):
    def __init__(self) -> None:
        super().__init__("stm32_bridge")

        self.declare_parameter("port", DEFAULT_BASE_PORT)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel/driver")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("cmd_timeout", 0.25)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("status_hz", 100.0)
        self.declare_parameter("serial_open_retry_sec", 2.0)
        self.declare_parameter("status_timeout", 0.75)
        self.declare_parameter("drive_keepalive_sec", 0.10)
        self.declare_parameter("startup_settle_sec", 0.20)
        self.declare_parameter("wheel_radius", 0.0350)
        self.declare_parameter("wheel_track_width", 0.1780)
        self.declare_parameter("odom_linear_scale", 1.0)
        self.declare_parameter("odom_angular_scale", 1.0)
        self.declare_parameter("odom_angular_sign", 1.0)
        self.declare_parameter("odom_linear_deadzone", 0.01)
        self.declare_parameter("odom_angular_deadzone", 0.03)
        self.declare_parameter("status_log_interval_sec", 0.0)
        self.declare_parameter("cmd_log_interval_sec", 0.0)

        self.port = self._normalize_configured_port(str(self.get_parameter("port").value))
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.status_hz = float(self.get_parameter("status_hz").value)
        self.serial_open_retry_sec = float(self.get_parameter("serial_open_retry_sec").value)
        self.status_timeout = float(self.get_parameter("status_timeout").value)
        self.drive_keepalive_sec = float(self.get_parameter("drive_keepalive_sec").value)
        self.startup_settle_sec = float(self.get_parameter("startup_settle_sec").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_track_width = float(self.get_parameter("wheel_track_width").value)
        self.odom_linear_scale = float(self.get_parameter("odom_linear_scale").value)
        self.odom_angular_scale = float(self.get_parameter("odom_angular_scale").value)
        self.odom_angular_sign = 1.0 if float(self.get_parameter("odom_angular_sign").value) >= 0.0 else -1.0
        self.odom_linear_deadzone = float(self.get_parameter("odom_linear_deadzone").value)
        self.odom_angular_deadzone = float(self.get_parameter("odom_angular_deadzone").value)
        self.status_log_interval_sec = float(self.get_parameter("status_log_interval_sec").value)
        self.cmd_log_interval_sec = float(self.get_parameter("cmd_log_interval_sec").value)

        self.serial = None
        self.connected_port = ""
        self.parser = FrameParser()
        self.last_open_attempt = 0.0
        self.last_missing_port_log = 0.0
        self.last_cmd_log_time = 0.0
        self.last_drive_log_time = 0.0
        self.last_status_log_time = 0.0
        self.warn_times: Dict[str, float] = {}

        self.command_stream = CommandStream(self.cmd_timeout, self.drive_keepalive_sec)
        self.has_seen_cmd_vel = False
        self.target_vx = 0.0
        self.target_wz = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_ts = self.get_clock().now()
        self.last_status_ts = None

        self.protocol_version = 0
        self.status_flags = 0
        self.control_source = 0
        self.motor_enabled_mask = 0
        self.motor_speed_valid_mask = 0
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

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None
        self.chassis_status_pub = self.create_publisher(UInt32, "/chassis/status", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.left_current_pub = self.create_publisher(Float32, "/motor/left_current", 10)
        self.right_current_pub = self.create_publisher(Float32, "/motor/right_current", 10)

        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 20)
        self.create_service(SetBool, "/chassis/estop", self.on_estop)

        self.create_timer(1.0 / max(self.control_hz, 1.0), self.control_loop)
        self.create_timer(1.0 / max(self.status_hz, 1.0), self.status_poll)

        self.ensure_serial(force_log_missing=True)
        self.get_logger().info(
            "STM32 bridge v2 started: "
            f"port={self.port} baud={self.baudrate} protocol={PROTOCOL_VERSION} "
            f"wheel_track_width={self.wheel_track_width:.4f} wheel_radius={self.wheel_radius:.4f} "
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
            self.get_logger().info(f"Opened STM32 serial {self.port}@{self.baudrate}")
            self.set_modem_lines_low()
            self.reset_serial_buffers()
            time.sleep(max(self.startup_settle_sec, 0.0))
            self.startup_release_controller()
            return True
        except Exception as exc:
            self.serial = None
            self.connected_port = ""
            self.get_logger().warn(f"Failed to open serial {self.port}: {exc}")
            return False

    def open_serial_port(self, resolved_port: str):
        ser = serial.Serial()
        ser.port = resolved_port
        ser.baudrate = self.baudrate
        ser.timeout = 0.0
        ser.write_timeout = 0.2
        ser.rtscts = False
        ser.dsrdtr = False
        try:
            ser.dtr = False
        except Exception:
            pass
        try:
            ser.rts = False
        except Exception:
            pass
        ser.open()
        try:
            ser.dtr = False
        except Exception:
            pass
        try:
            ser.rts = False
        except Exception:
            pass
        return ser

    def set_modem_lines_low(self) -> None:
        if self.serial is None:
            return
        for attr in ("dtr", "rts"):
            try:
                setattr(self.serial, attr, False)
            except Exception:
                pass

    def reset_serial_buffers(self) -> None:
        if self.serial is None:
            return
        for method_name in ("reset_input_buffer", "reset_output_buffer"):
            try:
                getattr(self.serial, method_name)()
            except Exception:
                pass

    def startup_release_controller(self) -> None:
        for _ in range(3):
            self.release_upper_control()
            time.sleep(0.02)

    def release_upper_control(self) -> bool:
        if self.serial is None:
            return False
        return self.command_stream.release(lambda payload: self.write_frame(CMD_SET_VELOCITY, payload))

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
        if reason:
            self.get_logger().warn(reason if not port else f"{reason} ({port})")

    def write_frame(self, cmd: int, payload: bytes) -> bool:
        if self.serial is None:
            return False
        try:
            self.serial.write(build_frame(cmd, payload))
            return True
        except Exception as exc:
            self.close_serial(f"Serial write failed: {exc}", release_control=False)
            return False

    def on_estop(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if not self.ensure_serial():
            response.success = False
            response.message = "serial not connected"
            return response
        enabled = bool(request.data)
        sent = self.write_frame(CMD_ESTOP, encode_estop_payload(enabled))
        response.success = sent
        response.message = "estop triggered" if enabled and sent else "estop released" if sent else "send failed"
        if sent:
            self.get_logger().warn("Sent ESTOP=1 to STM32" if enabled else "Sent ESTOP=0 to STM32")
        return response

    def on_cmd_vel(self, msg: Twist) -> None:
        self.has_seen_cmd_vel = True
        self.target_vx = float(msg.linear.x)
        self.target_wz = float(msg.angular.z)
        self.command_stream.update_command(self.target_vx, self.target_wz, time.monotonic())
        self.log_cmd_vel_rx(self.target_vx, self.target_wz)

    def control_loop(self) -> None:
        self.ensure_serial()
        self.read_serial_frames()

        now_sec = time.monotonic()
        self.command_stream.tick(now_sec, lambda payload: self.write_drive_payload(payload, now_sec))
        self.publish_odom(self.get_clock().now())

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
        self.read_serial_frames()

    def read_serial_frames(self) -> None:
        if self.serial is None:
            return
        try:
            waiting = self.serial.in_waiting
            if waiting <= 0:
                return
            chunk = self.serial.read(waiting)
        except Exception as exc:
            self.close_serial(f"Serial read failed: {exc}", release_control=False)
            return

        for cmd, payload in self.parser.feed(chunk):
            self.handle_frame(cmd, payload)

    def handle_frame(self, cmd: int, payload: bytes) -> None:
        if cmd != CMD_STATUS:
            self.get_logger().debug(f"Unknown frame cmd=0x{cmd:02X} len={len(payload)}")
            return
        if len(payload) != STATUS_PAYLOAD_SIZE:
            self.warn_periodic("bad_status_len", f"Discard STATUS payload len={len(payload)} expected={STATUS_PAYLOAD_SIZE}")
            return

        status = decode_status_payload(payload)
        if status is None:
            version = payload[0] if payload else -1
            self.warn_periodic("bad_status_version", f"Discard STATUS protocol version={version} expected={PROTOCOL_VERSION}")
            return

        agg = aggregate_status(status, self.wheel_track_width)
        self.last_status_ts = self.get_clock().now()
        self.protocol_version = status.version
        self.status_flags = status.status_flags
        self.control_source = status.control_source
        self.motor_enabled_mask = status.motor_enabled_mask
        self.motor_speed_valid_mask = status.motor_speed_valid_mask
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

        self.publish_auxiliary_topics()
        self.log_status_summary()

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

    def publish_odom(self, now) -> None:
        dt = (now - self.last_odom_ts).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_odom_ts = now

        status_fresh = False
        if self.last_status_ts is not None:
            age = (now - self.last_status_ts).nanoseconds * 1e-9
            status_fresh = 0.0 <= age <= self.status_timeout

        status_allows_motion = (self.status_flags & (STATUS_FLAG_ESTOP | STATUS_FLAG_FAULT_STOP)) == 0
        odom_trusted = status_fresh and self.feedback_odom_trusted and status_allows_motion
        odom_vx = self.apply_deadzone(self.feedback_vx if odom_trusted else 0.0, self.odom_linear_deadzone)
        odom_wz = self.apply_deadzone(self.feedback_wz if odom_trusted else 0.0, self.odom_angular_deadzone)

        yaw_delta = odom_wz * dt
        yaw_mid = self.yaw + 0.5 * yaw_delta
        self.x += odom_vx * math.cos(yaw_mid) * dt
        self.y += odom_vx * math.sin(yaw_mid) * dt
        self.yaw = _wrap_angle(self.yaw + yaw_delta)

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

        if odom_trusted:
            odom.pose.covariance[0] = 0.05
            odom.pose.covariance[7] = 0.05
            odom.pose.covariance[35] = 0.1
            odom.twist.covariance[0] = 0.05
            odom.twist.covariance[35] = 0.1
        else:
            odom.pose.covariance[0] = 1.0
            odom.pose.covariance[7] = 1.0
            odom.pose.covariance[35] = 3.14
            odom.twist.covariance[0] = 1000.0
            odom.twist.covariance[35] = 1000.0

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
        if abs(value) < max(threshold, 0.0):
            return 0.0
        return value

    def warn_periodic(self, key: str, message: str, interval_sec: float = 2.0) -> None:
        now_monotonic = time.monotonic()
        if (now_monotonic - self.warn_times.get(key, 0.0)) >= interval_sec:
            self.warn_times[key] = now_monotonic
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
