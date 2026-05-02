#!/usr/bin/env python3
import glob
import math
import os
import struct
import time
from typing import List, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float32, UInt32
from tf2_ros import TransformBroadcaster

try:
    import serial
except Exception:  # pragma: no cover
    serial = None

# AGENTS.md v1 protocol constants
FRAME_SOF = 0xA5
FRAME_SOF2 = 0x5A

CMD_SET_VELOCITY = 0x01
CMD_ESTOP = 0x02
CMD_STATUS = 0x81

# STATUS payload: left_speed(f32) + right_speed(f32)
#   + left_enc(i32) + right_enc(i32)
#   + battery_voltage(f32) + left_current(f32) + right_current(f32)
#   + imu_accel[3](i16) + imu_gyro[3](i16)
#   + error_flags(u32) + control_mode(u8)
STATUS_PAYLOAD_SIZE = 47  # 4+4+4+4+4+4+4+6+6+4+1

MODE_IDLE = 0
MODE_OPEN_LOOP = 1
MODE_CLOSED_LOOP = 2


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _safe_read_text(path: str) -> str:
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            return handle.read().strip()
    except OSError:
        return ''


def _find_usb_ids(sys_path: str) -> Tuple[str, str]:
    current = os.path.realpath(sys_path)
    for _ in range(8):
        vendor = _safe_read_text(os.path.join(current, 'idVendor')).lower()
        product = _safe_read_text(os.path.join(current, 'idProduct')).lower()
        if vendor:
            return vendor, product
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return '', ''


def _normalize_device_path(path: str) -> str:
    if not path:
        return ''
    resolved = os.path.realpath(path)
    if resolved.startswith('/dev/'):
        return resolved
    return path


def detect_cp210x_ports(excluded_ports: Optional[Set[str]] = None) -> List[str]:
    candidates: List[str] = []
    seen = set()
    excluded = {_normalize_device_path(port) for port in (excluded_ports or set()) if port}

    def add_port(port: str) -> None:
        resolved = _normalize_device_path(port)
        if resolved in seen or resolved in excluded or not os.path.exists(resolved):
            return
        seen.add(resolved)
        candidates.append(resolved)

    by_id_patterns = (
        '/dev/serial/by-id/*CP210*',
        '/dev/serial/by-id/*cp210*',
        '/dev/serial/by-id/*Silicon_Labs*',
        '/dev/serial/by-id/*USB*UART*',
    )
    for pattern in by_id_patterns:
        for entry in sorted(glob.glob(pattern)):
            add_port(entry)

    for port in sorted(glob.glob('/dev/ttyUSB*')):
        tty_name = os.path.basename(port)
        sys_device = os.path.join('/sys/class/tty', tty_name, 'device')
        driver = os.path.realpath(os.path.join(sys_device, 'driver')).lower()
        vendor, product = _find_usb_ids(sys_device)
        if 'cp210x' in driver or vendor == '10c4' or product == 'ea60':
            add_port(port)

    if candidates:
        return candidates

    tty_ports = sorted(glob.glob('/dev/ttyUSB*'))
    if len(tty_ports) == 1:
        return tty_ports
    return []


class Protocol:
    @staticmethod
    def build_frame(cmd: int, payload: bytes) -> bytes:
        length = len(payload)
        checksum = (length + cmd + sum(payload)) & 0xFF
        return bytes([FRAME_SOF, FRAME_SOF2, length, cmd]) + payload + bytes([checksum])

    @staticmethod
    def find_frame(buf: bytearray) -> Tuple[Optional[Tuple[int, bytes]], int]:
        """Search buf for a valid v1 frame. Returns ((cmd, payload), bytes_consumed)."""
        i = 0
        while i + 3 < len(buf):
            if buf[i] == FRAME_SOF and buf[i + 1] == FRAME_SOF2:
                length = buf[i + 2]
                frame_end = i + 4 + length + 1  # header(4) + payload + checksum(1)
                if frame_end > len(buf):
                    return None, i  # incomplete, wait for more data
                cmd = buf[i + 3]
                payload = bytes(buf[i + 4:i + 4 + length])
                recv_cs = buf[i + 4 + length]
                calc_cs = (length + cmd + sum(payload)) & 0xFF
                if recv_cs == calc_cs:
                    return (cmd, payload), frame_end
                # checksum mismatch, skip this header and keep searching
                i += 1
            else:
                i += 1
        return None, i


class STM32Bridge(Node):
    def __init__(self) -> None:
        super().__init__('stm32_bridge')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_imu', False)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('imu_frame_id', 'imu_link')

        self.declare_parameter('cmd_timeout', 0.25)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('status_hz', 20.0)
        self.declare_parameter('serial_open_retry_sec', 2.0)
        self.declare_parameter('status_timeout', 0.75)
        self.declare_parameter('drive_keepalive_sec', 0.10)
        self.declare_parameter('startup_settle_sec', 0.20)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_track_width', 0.1250)
        self.declare_parameter('odom_linear_scale', 1.0)
        self.declare_parameter('odom_angular_scale', 1.0)
        self.declare_parameter('odom_angular_sign', 1.0)
        self.declare_parameter('odom_linear_deadzone', 0.01)
        self.declare_parameter('odom_angular_deadzone', 0.03)
        self.declare_parameter('status_log_interval_sec', 0.0)
        self.declare_parameter('cmd_log_interval_sec', 0.0)
        self.declare_parameter('excluded_ports', [])

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_imu = bool(self.get_parameter('publish_imu').value)
        self.imu_topic = self.get_parameter('imu_topic').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.status_hz = float(self.get_parameter('status_hz').value)
        self.serial_open_retry_sec = float(self.get_parameter('serial_open_retry_sec').value)
        self.status_timeout = float(self.get_parameter('status_timeout').value)
        self.drive_keepalive_sec = float(self.get_parameter('drive_keepalive_sec').value)
        self.startup_settle_sec = float(self.get_parameter('startup_settle_sec').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_track_width = float(self.get_parameter('wheel_track_width').value)
        self.odom_linear_scale = float(self.get_parameter('odom_linear_scale').value)
        self.odom_angular_scale = float(self.get_parameter('odom_angular_scale').value)
        self.odom_angular_sign = 1.0 if float(self.get_parameter('odom_angular_sign').value) >= 0.0 else -1.0
        self.odom_linear_deadzone = float(self.get_parameter('odom_linear_deadzone').value)
        self.odom_angular_deadzone = float(self.get_parameter('odom_angular_deadzone').value)
        self.status_log_interval_sec = float(self.get_parameter('status_log_interval_sec').value)
        self.cmd_log_interval_sec = float(self.get_parameter('cmd_log_interval_sec').value)
        excluded_ports_param = self.get_parameter('excluded_ports').value
        if isinstance(excluded_ports_param, str):
            excluded_ports_list = [item.strip() for item in excluded_ports_param.split(',') if item.strip()]
        else:
            excluded_ports_list = [str(item).strip() for item in excluded_ports_param if str(item).strip()]
        env_excluded_ports = [
            os.environ.get('ROBOT_LIDAR_PORT_HINT', '').strip(),
            os.environ.get('LIDAR_PORT_HINT', '').strip(),
        ]
        self.excluded_ports = {
            _normalize_device_path(port)
            for port in excluded_ports_list + env_excluded_ports
            if port
        }

        self.serial = None
        self.connected_port = ''
        self.rx_buf = bytearray()
        self.last_open_attempt = 0.0
        self.last_missing_port_log = 0.0
        self.last_probe_candidates: List[str] = []
        self.last_drive_tx_time = 0.0
        self.last_drive_cmd: Optional[Tuple[float, float]] = None
        self.status_mode = MODE_IDLE
        self.last_mode_warn_time = 0.0
        self.last_cmd_log_time = 0.0
        self.last_drive_log_time = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.has_seen_cmd_vel = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_ts = self.get_clock().now()
        self.last_status_ts = None
        self.feedback_vx = 0.0
        self.feedback_wz = 0.0
        self.feedback_left_speed = 0.0
        self.feedback_right_speed = 0.0
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.feedback_battery_voltage = 0.0
        self.feedback_left_current = 0.0
        self.feedback_right_current = 0.0
        self.feedback_error_flags = 0
        self.feedback_imu_valid = False
        self.feedback_raw_accel = (0.0, 0.0, 0.0)
        self.feedback_gz = 0.0
        self.feedback_vx_wheels = 0.0
        self.feedback_wz_wheels = 0.0
        self.status_revision = 0
        self.last_status_log_time = 0.0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 20) if self.publish_imu else None
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None
        self.chassis_status_pub = self.create_publisher(UInt32, '/chassis/status', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.left_current_pub = self.create_publisher(Float32, '/motor/left_current', 10)
        self.right_current_pub = self.create_publisher(Float32, '/motor/right_current', 10)

        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 20)

        self.create_timer(1.0 / max(self.control_hz, 1.0), self.control_loop)
        self.create_timer(1.0 / max(self.status_hz, 1.0), self.status_poll)

        self.ensure_serial(force_log_missing=True)
        self.get_logger().info(
            f'STM32 bridge v1 started: port={self.port} baud={self.baudrate} '
            f'publish_tf={self.publish_tf} publish_imu={self.publish_imu}'
        )

    def resolve_port(self) -> Optional[str]:
        requested = self.port.strip()
        if requested and requested.lower() != 'auto':
            return requested

        candidates = detect_cp210x_ports(self.excluded_ports)
        self.last_probe_candidates = candidates
        if not candidates:
            return None
        for candidate in candidates:
            if self.probe_port_for_status(candidate):
                return candidate
        return None

    def probe_port_for_status(self, resolved_port: str) -> bool:
        if serial is None:
            return False

        try:
            probe_serial = serial.Serial(
                port=resolved_port,
                baudrate=self.baudrate,
                timeout=0.2,
                write_timeout=0.2,
                rtscts=False,
                dsrdtr=False,
            )
        except Exception:
            return False

        try:
            for attr in ('dtr', 'rts'):
                try:
                    setattr(probe_serial, attr, False)
                except Exception:
                    pass
            for method_name in ('reset_input_buffer', 'reset_output_buffer'):
                try:
                    getattr(probe_serial, method_name)()
                except Exception:
                    pass

            time.sleep(max(self.startup_settle_sec, 0.0))
            rx_buf = bytearray()
            for _ in range(3):
                frame = Protocol.build_frame(CMD_STATUS, b'')
                try:
                    probe_serial.write(frame)
                    probe_serial.flush()
                except Exception:
                    return False

                deadline = time.monotonic() + 0.35
                while time.monotonic() < deadline:
                    try:
                        chunk = probe_serial.read(256)
                    except Exception:
                        return False
                    if chunk:
                        rx_buf.extend(chunk)
                    else:
                        time.sleep(0.03)

                    result, consumed = Protocol.find_frame(rx_buf)
                    if result is not None:
                        cmd, payload = result
                        del rx_buf[:consumed]
                        if cmd == CMD_STATUS and len(payload) >= STATUS_PAYLOAD_SIZE:
                            return True
                    else:
                        # Discard bytes before the search position to prevent unbounded growth
                        if consumed > 0:
                            del rx_buf[:consumed]
            return False
        finally:
            try:
                probe_serial.close()
            except Exception:
                pass

    def close_serial(self, reason: str) -> None:
        port = self.connected_port
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.connected_port = ''
        self.rx_buf.clear()
        self.last_drive_cmd = None
        self.last_drive_tx_time = 0.0
        if reason:
            self.get_logger().warn(reason if not port else f'{reason} ({port})')

    def ensure_serial(self, force_log_missing: bool = False) -> bool:
        if self.serial is not None:
            return True
        if serial is None:
            self.get_logger().error('python3-serial not installed')
            return False

        now_monotonic = time.monotonic()
        if not force_log_missing and (now_monotonic - self.last_open_attempt) < self.serial_open_retry_sec:
            return False
        self.last_open_attempt = now_monotonic

        resolved_port = self.resolve_port()
        if resolved_port is None:
            if force_log_missing or (now_monotonic - self.last_missing_port_log) >= 5.0:
                if self.port.strip().lower() == 'auto':
                    if self.last_probe_candidates:
                        self.get_logger().warn(
                            'Detected CP2102 ports but none replied to GET_STATUS yet; '
                            f'bridge will keep retrying: {", ".join(self.last_probe_candidates)}'
                        )
                    else:
                        self.get_logger().warn('No CP2102 serial port detected yet; bridge will keep retrying')
                else:
                    self.get_logger().warn(f'Serial port {self.port} not available yet; bridge will keep retrying')
                self.last_missing_port_log = now_monotonic
            return False

        try:
            self.serial = self.open_serial_port(resolved_port)
            self.connected_port = resolved_port
            self.rx_buf.clear()
            self.target_vx = 0.0
            self.target_wz = 0.0
            self.has_seen_cmd_vel = False
            self.last_cmd_time = self.get_clock().now()
            self.last_drive_cmd = None
            self.last_drive_tx_time = 0.0
            self.get_logger().info(f'Opened STM32 serial {resolved_port}@{self.baudrate}')
            self.set_modem_lines_low()
            self.reset_serial_buffers()
            time.sleep(max(self.startup_settle_sec, 0.0))
            self.startup_sync_controller()
            self.write_frame(CMD_STATUS, b'')
            return True
        except Exception as e:
            self.serial = None
            self.connected_port = ''
            self.get_logger().warn(f'Failed to open serial {resolved_port}: {e}')
            return False

    def write_frame(self, cmd: int, payload: bytes) -> bool:
        if self.serial is None:
            return False
        frame = Protocol.build_frame(cmd, payload)
        try:
            self.serial.write(frame)
            return True
        except Exception as e:
            self.close_serial(f'Serial write failed: {e}')
            return False

    def startup_sync_controller(self) -> None:
        zero_payload = self.encode_drive_payload(0.0, 0.0)
        for _ in range(3):
            self.write_frame(CMD_SET_VELOCITY, zero_payload)
            time.sleep(0.02)

    def encode_drive_payload(self, vx: float, wz: float) -> bytes:
        return struct.pack('<ffBB', vx, wz, 1, MODE_CLOSED_LOOP)

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

    @staticmethod
    def apply_deadzone(value: float, threshold: float) -> float:
        if abs(value) < max(threshold, 0.0):
            return 0.0
        return value


    def log_status_summary(self) -> None:
        if self.status_log_interval_sec <= 0.0:
            return
        now_monotonic = time.monotonic()
        if (now_monotonic - self.last_status_log_time) < self.status_log_interval_sec:
            return
        self.last_status_log_time = now_monotonic
        self.get_logger().info(
            'status_summary '
            f'mode={self.status_mode} '
            f'odom=({self.feedback_vx:.3f}m/s,{math.degrees(self.feedback_wz):.2f}deg/s) '
            f'wheels=({self.feedback_left_speed:.3f},{self.feedback_right_speed:.3f})m/s '
            f'bat={self.feedback_battery_voltage:.2f}V '
            f'cur=({self.feedback_left_current:.2f},{self.feedback_right_current:.2f})A '
            f'err=0x{self.feedback_error_flags:08X}'
        )

    def set_modem_lines_low(self) -> None:
        if self.serial is None:
            return
        for attr in ('dtr', 'rts'):
            try:
                setattr(self.serial, attr, False)
            except Exception:
                pass

    def reset_serial_buffers(self) -> None:
        if self.serial is None:
            return
        for method_name in ('reset_input_buffer', 'reset_output_buffer'):
            try:
                getattr(self.serial, method_name)()
            except Exception:
                pass

    def on_cmd_vel(self, msg: Twist) -> None:
        self.has_seen_cmd_vel = True
        self.target_vx = float(msg.linear.x)
        self.target_wz = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()
        self.log_cmd_vel_rx(self.target_vx, self.target_wz)

    def control_loop(self) -> None:
        self.ensure_serial()
        self.read_serial_frames()

        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        cmd_active = dt <= self.cmd_timeout
        if not cmd_active:
            vx = 0.0
            wz = 0.0
        else:
            vx = self.target_vx
            wz = self.target_wz

        payload = self.encode_drive_payload(vx, wz)
        drive_key = (vx, wz)
        now_monotonic = time.monotonic()
        should_send = self.last_drive_cmd != drive_key
        if cmd_active:
            should_send = (
                should_send
                or (now_monotonic - self.last_drive_tx_time) >= max(self.drive_keepalive_sec, 0.05)
            )
        if self.has_seen_cmd_vel and should_send:
            sent = self.write_frame(CMD_SET_VELOCITY, payload)
            self.log_drive_tx(vx, wz, sent, cmd_active)
            if sent:
                self.last_drive_cmd = drive_key
                self.last_drive_tx_time = now_monotonic

        self.publish_odom(now)

    def log_cmd_vel_rx(self, vx: float, wz: float) -> None:
        if self.cmd_log_interval_sec <= 0.0:
            return
        now_monotonic = time.monotonic()
        if (now_monotonic - self.last_cmd_log_time) < self.cmd_log_interval_sec:
            return
        self.last_cmd_log_time = now_monotonic
        status_age = 'none'
        if self.last_status_ts is not None:
            age = (self.get_clock().now() - self.last_status_ts).nanoseconds * 1e-9
            status_age = f'{age:.2f}s'
        self.get_logger().info(
            'cmd_vel_rx '
            f'cmd=({vx:.3f}m/s,{math.degrees(wz):.2f}deg/s) '
            f'serial={self.connected_port or "none"} '
            f'mode={self.status_mode} status_age={status_age}'
        )

    def log_drive_tx(
        self,
        vx: float,
        wz: float,
        sent: bool,
        cmd_active: bool,
    ) -> None:
        if self.cmd_log_interval_sec <= 0.0:
            return
        now_monotonic = time.monotonic()
        if (now_monotonic - self.last_drive_log_time) < self.cmd_log_interval_sec:
            return
        self.last_drive_log_time = now_monotonic
        self.get_logger().info(
            'drive_tx '
            f'sent={1 if sent else 0} active={1 if cmd_active else 0} '
            f'cmd=({vx:.3f}m/s,{math.degrees(wz):.2f}deg/s) '
            f'serial={self.connected_port or "none"} '
            f'mode={self.status_mode}'
        )

    def status_poll(self) -> None:
        if not self.ensure_serial():
            return
        self.read_serial_frames()
        self.write_frame(CMD_STATUS, b'')

    def warn_if_not_closed_loop(self, mode: int) -> None:
        if mode == MODE_CLOSED_LOOP or not self.has_seen_cmd_vel:
            return
        now_monotonic = time.monotonic()
        if (now_monotonic - self.last_mode_warn_time) >= 2.0:
            self.get_logger().warn(
                f'STM32 reports control_mode={mode}; drive requires closed_loop ({MODE_CLOSED_LOOP})'
            )
            self.last_mode_warn_time = now_monotonic

    def read_serial_frames(self) -> None:
        if self.serial is None:
            return
        try:
            waiting = self.serial.in_waiting
            if waiting > 0:
                self.rx_buf.extend(self.serial.read(waiting))
        except Exception as e:
            self.close_serial(f'Serial read failed: {e}')
            return

        while True:
            result, consumed = Protocol.find_frame(self.rx_buf)
            if result is not None:
                del self.rx_buf[:consumed]
                self.handle_frame(*result)
            else:
                # Discard bytes before the search position to prevent unbounded growth
                if consumed > 0:
                    del self.rx_buf[:consumed]
                break

    def handle_frame(self, cmd: int, payload: bytes) -> None:
        if cmd != CMD_STATUS:
            self.get_logger().debug(f'Unknown frame cmd=0x{cmd:02X} len={len(payload)}')
            return

        if len(payload) < STATUS_PAYLOAD_SIZE:
            self.get_logger().warn(f'STATUS payload too short: {len(payload)} < {STATUS_PAYLOAD_SIZE}')
            return

        # v1 STATUS payload layout (47 bytes):
        #   left_speed(f32) right_speed(f32) left_enc(i32) right_enc(i32)
        #   battery_voltage(f32) left_current(f32) right_current(f32)
        #   imu_accel[3](i16) imu_gyro[3](i16)
        #   error_flags(u32) control_mode(u8)
        (
            left_speed, right_speed,
            left_enc, right_enc,
            battery_voltage,
            left_current, right_current,
            accel_x, accel_y, accel_z,
            gyro_x, gyro_y, gyro_z,
            error_flags,
            control_mode,
        ) = struct.unpack_from('<ff i i f f f h h h h h h I B', payload, 0)

        self.status_mode = control_mode
        self.feedback_left_speed = left_speed
        self.feedback_right_speed = right_speed
        self.last_left_encoder = left_enc
        self.last_right_encoder = right_enc
        self.feedback_battery_voltage = battery_voltage
        self.feedback_left_current = left_current
        self.feedback_right_current = right_current
        self.feedback_error_flags = error_flags

        # IMU: raw i16 → physical units
        # Accelerometer: ±16g range, i16 → m/s²  (scale = 16*9.80665/32768)
        accel_scale = 16.0 * 9.80665 / 32768.0
        self.feedback_raw_accel = (
            accel_x * accel_scale,
            accel_y * accel_scale,
            accel_z * accel_scale,
        )
        # Gyroscope: ±2000°/s range, i16 → rad/s  (scale = 2000*π/(180*32768))
        gyro_scale = 2000.0 * math.pi / (180.0 * 32768.0)
        self.feedback_gz = gyro_z * gyro_scale
        self.feedback_imu_valid = True

        self.last_status_ts = self.get_clock().now()

        # Compute wheel-based odometry from wheel speeds (m/s from MCU)
        vx = 0.5 * (left_speed + right_speed)
        wz = (right_speed - left_speed) / max(self.wheel_track_width, 1e-6)
        self.feedback_vx_wheels = vx * self.odom_linear_scale
        self.feedback_wz_wheels = wz * self.odom_angular_scale * self.odom_angular_sign

        self.feedback_vx = self.feedback_vx_wheels
        self.feedback_wz = self.feedback_wz_wheels

        self.status_revision += 1

        # Publish auxiliary topics
        self.chassis_status_pub.publish(UInt32(data=error_flags))

        batt = BatteryState()
        batt.header.stamp = self.last_status_ts.to_msg()
        batt.voltage = battery_voltage
        batt.present = battery_voltage > 1.0
        self.battery_pub.publish(batt)

        self.left_current_pub.publish(Float32(data=left_current))
        self.right_current_pub.publish(Float32(data=right_current))

        self.get_logger().debug(
            f'status mode={control_mode} '
            f'wheel_l={left_speed:.3f}m/s wheel_r={right_speed:.3f}m/s '
            f'enc_l={left_enc} enc_r={right_enc} '
            f'bat={battery_voltage:.2f}V '
            f'cur_l={left_current:.2f}A cur_r={right_current:.2f}A '
            f'acc=({accel_x},{accel_y},{accel_z}) gyro=({gyro_x},{gyro_y},{gyro_z}) '
            f'err=0x{error_flags:08X}'
        )
        self.log_status_summary()
        self.warn_if_not_closed_loop(control_mode)

    def publish_odom(self, now) -> None:
        dt = (now - self.last_odom_ts).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_odom_ts = now

        status_fresh = False
        if self.last_status_ts is not None:
            age = (now - self.last_status_ts).nanoseconds * 1e-9
            status_fresh = 0.0 <= age <= self.status_timeout

        odom_vx = self.feedback_vx if status_fresh else 0.0
        odom_wz = self.feedback_wz if status_fresh else 0.0

        odom_vx = self.apply_deadzone(odom_vx, self.odom_linear_deadzone)
        odom_wz = self.apply_deadzone(odom_wz, self.odom_angular_deadzone)

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

        if status_fresh:
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
        self.publish_imu_msg(now)

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

    def publish_imu_msg(self, now) -> None:
        if self.imu_pub is None or self.last_status_ts is None:
            return

        age = (now - self.last_status_ts).nanoseconds * 1e-9
        if age < 0.0 or age > self.status_timeout:
            return

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.imu_frame_id

        # Orientation: not estimated by bridge; EKF fuses gyro integration
        imu.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 1e6,
        ]

        imu.angular_velocity.z = self.feedback_gz
        imu.angular_velocity_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 0.02,
        ]

        ax, ay, az = self.feedback_raw_accel
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.linear_acceleration_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05,
        ]

        self.imu_pub.publish(imu)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = STM32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial is not None:
            try:
                node.serial.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
