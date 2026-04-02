#!/usr/bin/env python3
import glob
import math
import os
import struct
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

try:
    import serial
except Exception:  # pragma: no cover
    serial = None


LINK_SOF = 0xA5
LINK_EOF = 0x5A
LINK_VER = 0x01

MSG_CMD_GET_STATUS = 0x02
MSG_CMD_SET_DRIVE = 0x10
MSG_CMD_SET_MODE = 0x11
MSG_CMD_SET_IMU = 0x12
MSG_ACK = 0x80
MSG_NACK = 0x81

FLAG_ACK_REQ = 0x01
FLAG_IS_ACK = 0x02

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


def detect_cp210x_ports() -> List[str]:
    candidates: List[str] = []
    seen = set()

    def add_port(port: str) -> None:
        resolved = os.path.realpath(port)
        if not resolved.startswith('/dev/'):
            resolved = port
        if resolved in seen or not os.path.exists(resolved):
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
    def crc16_ccitt_false(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= (b << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def cobs_encode(raw: bytes) -> bytes:
        out = bytearray()
        code_idx = 0
        out.append(0)
        code = 1
        for b in raw:
            if b == 0:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
            else:
                out.append(b)
                code += 1
                if code == 0xFF:
                    out[code_idx] = code
                    code_idx = len(out)
                    out.append(0)
                    code = 1
        out[code_idx] = code
        return bytes(out)

    @staticmethod
    def cobs_decode(enc: bytes) -> Optional[bytes]:
        out = bytearray()
        i = 0
        n = len(enc)
        while i < n:
            code = enc[i]
            i += 1
            if code == 0:
                return None
            for _ in range(code - 1):
                if i >= n:
                    return None
                out.append(enc[i])
                i += 1
            if code != 0xFF and i < n:
                out.append(0)
        return bytes(out)

    @staticmethod
    def build_frame(msg_type: int, flags: int, seq: int, payload: bytes) -> bytes:
        header = bytes([LINK_SOF, LINK_VER, msg_type, flags, seq]) + struct.pack('<H', len(payload))
        crc = Protocol.crc16_ccitt_false(header[1:] + payload)
        frame = header + payload + struct.pack('<H', crc) + bytes([LINK_EOF])
        return Protocol.cobs_encode(frame) + b'\x00'

    @staticmethod
    def parse_frame(raw: bytes) -> Optional[Tuple[int, int, int, bytes]]:
        dec = Protocol.cobs_decode(raw)
        if dec is None or len(dec) < 10:
            return None
        if dec[0] != LINK_SOF or dec[-1] != LINK_EOF:
            return None

        ver = dec[1]
        msg_type = dec[2]
        flags = dec[3]
        seq = dec[4]
        payload_len = struct.unpack_from('<H', dec, 5)[0]
        expect_len = payload_len + 10
        if len(dec) != expect_len:
            return None

        payload = dec[7:7 + payload_len]
        recv_crc = struct.unpack_from('<H', dec, 7 + payload_len)[0]
        calc_crc = Protocol.crc16_ccitt_false(dec[1:7] + payload)
        if recv_crc != calc_crc or ver != LINK_VER:
            return None
        return msg_type, flags, seq, payload


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
        self.declare_parameter('imu_frame_id', 'base_link')

        self.declare_parameter('max_linear', 0.50)
        self.declare_parameter('max_angular', 1.50)
        self.declare_parameter('cmd_timeout', 0.30)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('status_hz', 20.0)
        self.declare_parameter('serial_open_retry_sec', 2.0)
        self.declare_parameter('status_timeout', 0.75)
        self.declare_parameter('drive_ack', True)
        self.declare_parameter('drive_keepalive_sec', 0.20)
        self.declare_parameter('imu_enabled', True)
        self.declare_parameter('startup_settle_sec', 0.20)
        self.declare_parameter('startup_stop_retries', 3)
        self.declare_parameter('use_status_yaw', False)
        self.declare_parameter('odom_linear_deadzone', 0.01)
        self.declare_parameter('odom_angular_deadzone', 0.03)

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

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.status_hz = float(self.get_parameter('status_hz').value)
        self.serial_open_retry_sec = float(self.get_parameter('serial_open_retry_sec').value)
        self.status_timeout = float(self.get_parameter('status_timeout').value)
        self.drive_ack = bool(self.get_parameter('drive_ack').value)
        self.drive_keepalive_sec = float(self.get_parameter('drive_keepalive_sec').value)
        self.imu_enabled = bool(self.get_parameter('imu_enabled').value)
        self.startup_settle_sec = float(self.get_parameter('startup_settle_sec').value)
        self.startup_stop_retries = int(self.get_parameter('startup_stop_retries').value)
        self.use_status_yaw = bool(self.get_parameter('use_status_yaw').value)
        self.odom_linear_deadzone = float(self.get_parameter('odom_linear_deadzone').value)
        self.odom_angular_deadzone = float(self.get_parameter('odom_angular_deadzone').value)

        self.serial = None
        self.connected_port = ''
        self.rx_buf = bytearray()
        self.seq = 0
        self.last_open_attempt = 0.0
        self.last_missing_port_log = 0.0
        self.last_probe_candidates: List[str] = []
        self.last_drive_tx_time = 0.0
        self.last_drive_cmd = None
        self.status_mode = MODE_IDLE
        self.last_mode_warn_time = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.has_seen_cmd_vel = False

        self.current_vx = 0.0
        self.current_wz = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_ts = self.get_clock().now()
        self.last_status_ts = None
        self.feedback_vx = 0.0
        self.feedback_wz = 0.0
        self.feedback_yaw = 0.0
        self.feedback_yaw_available = False
        self.feedback_gz = 0.0
        self.feedback_raw_accel = (0.0, 0.0, 0.0)
        self.feedback_imu_valid = False
        self.feedback_imu_accel_valid = False
        self.last_encoder_cps = (0, 0, 0, 0)
        self.status_revision = 0
        self.applied_status_revision = 0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 20) if self.publish_imu else None
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 20)

        self.create_timer(1.0 / max(self.control_hz, 1.0), self.control_loop)
        self.create_timer(1.0 / max(self.status_hz, 1.0), self.status_poll)

        self.ensure_serial(force_log_missing=True)
        self.get_logger().info(f'STM32 bridge started with port={self.port} baudrate={self.baudrate}')

    def resolve_port(self) -> Optional[str]:
        requested = self.port.strip()
        if requested and requested.lower() != 'auto':
            return requested

        candidates = detect_cp210x_ports()
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
            for seq in range(1, 3):
                frame = Protocol.build_frame(MSG_CMD_GET_STATUS, FLAG_ACK_REQ, seq, b'')
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

                    while True:
                        try:
                            idx = rx_buf.index(0)
                        except ValueError:
                            break

                        raw = bytes(rx_buf[:idx])
                        del rx_buf[:idx + 1]
                        if not raw:
                            continue

                        parsed = Protocol.parse_frame(raw)
                        if parsed is None:
                            continue

                        msg_type, flags, _rx_seq, payload = parsed
                        if msg_type == MSG_ACK and len(payload) >= 22:
                            return True
                        if msg_type == MSG_CMD_GET_STATUS and (flags & FLAG_IS_ACK) and len(payload) >= 18:
                            return True
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
            self.current_vx = 0.0
            self.current_wz = 0.0
            self.has_seen_cmd_vel = False
            self.last_cmd_time = self.get_clock().now()
            self.last_drive_cmd = None
            self.last_drive_tx_time = 0.0
            self.get_logger().info(f'Opened STM32 serial {resolved_port}@{self.baudrate}')
            self.set_modem_lines_low()
            self.reset_serial_buffers()
            time.sleep(max(self.startup_settle_sec, 0.0))
            self.startup_sync_controller()
            self.send_set_imu(self.imu_enabled)
            self.write_frame(MSG_CMD_GET_STATUS, b'', ack_req=True)
            return True
        except Exception as e:
            self.serial = None
            self.connected_port = ''
            self.get_logger().warn(f'Failed to open serial {resolved_port}: {e}')
            return False

    def next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def write_frame(self, msg_type: int, payload: bytes, ack_req: bool = True) -> bool:
        if self.serial is None:
            return False
        flags = FLAG_ACK_REQ if ack_req else 0
        frame = Protocol.build_frame(msg_type, flags, self.next_seq(), payload)
        try:
            self.serial.write(frame)
            return True
        except Exception as e:
            self.close_serial(f'Serial write failed: {e}')
            return False

    def startup_sync_controller(self) -> None:
        zero_drive = self.encode_drive_payload(0.0, 0.0)
        retries = max(self.startup_stop_retries, 1)
        for _ in range(retries):
            self.send_set_mode(MODE_CLOSED_LOOP)
            self.write_frame(MSG_CMD_SET_DRIVE, zero_drive, ack_req=self.drive_ack)
            time.sleep(0.02)

    def encode_drive_payload(self, vx: float, wz: float) -> bytes:
        v_norm = max(-1.0, min(1.0, vx / max(self.max_linear, 1e-6)))
        w_norm = max(-1.0, min(1.0, wz / max(self.max_angular, 1e-6)))

        v_q15 = int(max(-32767, min(32767, round(v_norm * 32767.0))))
        w_q15 = int(max(-32767, min(32767, round(w_norm * 32767.0))))
        return struct.pack('<hh', v_q15, w_q15)

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

        self.current_vx = vx
        self.current_wz = wz

        payload = self.encode_drive_payload(vx, wz)
        drive_cmd = struct.unpack('<hh', payload)
        now_monotonic = time.monotonic()
        should_send_drive = self.last_drive_cmd != drive_cmd
        if cmd_active:
            should_send_drive = (
                should_send_drive
                or (now_monotonic - self.last_drive_tx_time) >= max(self.drive_keepalive_sec, 0.05)
            )
        if self.has_seen_cmd_vel and should_send_drive:
            if self.write_frame(MSG_CMD_SET_DRIVE, payload, ack_req=self.drive_ack):
                self.last_drive_cmd = drive_cmd
                self.last_drive_tx_time = now_monotonic

        self.publish_odom(now)

    def status_poll(self) -> None:
        if not self.ensure_serial():
            return
        self.read_serial_frames()
        self.write_frame(MSG_CMD_GET_STATUS, b'', ack_req=True)

    def send_set_imu(self, enabled: bool) -> None:
        self.write_frame(MSG_CMD_SET_IMU, bytes([1 if enabled else 0]), ack_req=True)

    def send_set_mode(self, mode: int) -> None:
        self.write_frame(MSG_CMD_SET_MODE, bytes([mode & 0xFF]), ack_req=True)

    def warn_if_not_closed_loop(self, mode: int) -> None:
        if mode == MODE_CLOSED_LOOP or not self.has_seen_cmd_vel:
            return
        now_monotonic = time.monotonic()
        if (now_monotonic - self.last_mode_warn_time) >= 2.0:
            self.get_logger().warn(
                f'STM32 reports mode={mode}; /cmd_vel drive requires MODE_CLOSED_LOOP ({MODE_CLOSED_LOOP})'
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
            try:
                idx = self.rx_buf.index(0)
            except ValueError:
                break

            raw = bytes(self.rx_buf[:idx])
            del self.rx_buf[:idx + 1]
            if not raw:
                continue

            parsed = Protocol.parse_frame(raw)
            if parsed is None:
                continue
            self.handle_frame(*parsed)

    def handle_frame(self, msg_type: int, _flags: int, _seq: int, payload: bytes) -> None:
        if msg_type == MSG_NACK:
            if len(payload) >= 4:
                ack_seq = payload[0]
                err_code = struct.unpack_from('<H', payload, 1)[0]
                self.get_logger().warn(f'STM32 NACK seq={ack_seq} err=0x{err_code:04X}')
            return

        if msg_type == MSG_ACK:
            if len(payload) <= 4:
                return
            ext = payload[4:]
        elif msg_type == MSG_CMD_GET_STATUS and (_flags & FLAG_IS_ACK):
            ext = payload
        else:
            return

        if len(ext) >= 40:
            (
                tick_ms,
                mode,
                src,
                vb_mv,
                pct_x10,
                v_q15,
                w_q15,
                gz_x10,
                imu_en,
                imu_valid,
                v_est_q15,
                w_est_q15,
                enc_fault_mask,
                vel_l1,
                vel_l2,
                vel_r1,
                vel_r2,
                yaw_est_x100,
                raw_ax_mg,
                raw_ay_mg,
                raw_az_mg,
                imu_accel_valid,
            ) = struct.unpack('<IBBHHhhhBBhhBhhhhhhhhB', ext[:40])
            self.status_mode = mode
            self.feedback_vx = (v_est_q15 / 32767.0) * self.max_linear
            self.feedback_wz = (w_est_q15 / 32767.0) * self.max_angular
            self.feedback_yaw = _wrap_angle(math.radians(yaw_est_x100 / 100.0))
            self.feedback_yaw_available = self.use_status_yaw
            self.feedback_gz = math.radians(gz_x10 / 10.0)
            self.feedback_raw_accel = (
                raw_ax_mg * 9.80665e-3,
                raw_ay_mg * 9.80665e-3,
                raw_az_mg * 9.80665e-3,
            )
            self.feedback_imu_valid = bool(imu_valid)
            self.feedback_imu_accel_valid = bool(imu_accel_valid)
            self.last_status_ts = self.get_clock().now()
            self.last_encoder_cps = (vel_l1, vel_l2, vel_r1, vel_r2)
            self.status_revision += 1
            self.get_logger().debug(
                f'status mode={mode} src={src} vb={vb_mv/1000.0:.2f}V pct={pct_x10/10.0:.1f}% '
                f'cmd=({v_q15/32767.0:.2f},{w_q15/32767.0:.2f}) '
                f'est=({v_est_q15/32767.0:.2f},{w_est_q15/32767.0:.2f}) '
                f'yaw={yaw_est_x100/100.0:.2f}deg g_z={gz_x10/10.0:.1f} imu={imu_en}/{imu_valid}/{imu_accel_valid} '
                f'acc=({raw_ax_mg},{raw_ay_mg},{raw_az_mg})mg '
                f'enc_fault=0x{enc_fault_mask:02X} tick={tick_ms}'
            )
            self.warn_if_not_closed_loop(mode)
            return

        if len(ext) >= 31:
            (
                tick_ms,
                mode,
                src,
                vb_mv,
                pct_x10,
                v_q15,
                w_q15,
                gz_x10,
                imu_en,
                imu_valid,
                v_est_q15,
                w_est_q15,
                enc_fault_mask,
                vel_l1,
                vel_l2,
                vel_r1,
                vel_r2,
            ) = struct.unpack('<IBBHHhhhBBhhBhhhh', ext[:31])
            self.status_mode = mode
            self.feedback_vx = (v_est_q15 / 32767.0) * self.max_linear
            self.feedback_wz = (w_est_q15 / 32767.0) * self.max_angular
            self.feedback_yaw_available = False
            self.feedback_gz = math.radians(gz_x10 / 10.0)
            self.feedback_raw_accel = (0.0, 0.0, 0.0)
            self.feedback_imu_valid = False
            self.feedback_imu_accel_valid = False
            self.last_status_ts = self.get_clock().now()
            self.last_encoder_cps = (vel_l1, vel_l2, vel_r1, vel_r2)
            self.status_revision += 1
            self.get_logger().debug(
                f'status mode={mode} src={src} vb={vb_mv/1000.0:.2f}V pct={pct_x10/10.0:.1f}% '
                f'cmd=({v_q15/32767.0:.2f},{w_q15/32767.0:.2f}) '
                f'est=({v_est_q15/32767.0:.2f},{w_est_q15/32767.0:.2f}) '
                f'g_z={gz_x10/10.0:.1f} imu={imu_en}/{imu_valid} '
                f'enc_fault=0x{enc_fault_mask:02X} tick={tick_ms}'
            )
            self.warn_if_not_closed_loop(mode)
            return

        if len(ext) >= 18:
            _, mode, src, vb_mv, pct_x10, v_q15, w_q15, _, imu_en, imu_valid = struct.unpack('<IBBHHhhhBB', ext[:18])
            self.status_mode = mode
            self.feedback_vx = (v_q15 / 32767.0) * self.max_linear
            self.feedback_wz = (w_q15 / 32767.0) * self.max_angular
            self.feedback_yaw_available = False
            self.feedback_gz = 0.0
            self.feedback_raw_accel = (0.0, 0.0, 0.0)
            self.feedback_imu_valid = False
            self.feedback_imu_accel_valid = False
            self.last_status_ts = self.get_clock().now()
            self.status_revision += 1
            self.get_logger().debug(
                f'legacy status mode={mode} src={src} vb={vb_mv/1000.0:.2f}V pct={pct_x10/10.0:.1f}% '
                f'cmd=({v_q15/32767.0:.2f},{w_q15/32767.0:.2f}) imu={imu_en}/{imu_valid}'
            )
            self.warn_if_not_closed_loop(mode)

    def publish_odom(self, now) -> None:
        dt = (now - self.last_odom_ts).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_odom_ts = now

        odom_vx = self.current_vx
        odom_wz = self.current_wz
        if self.last_status_ts is not None:
            age = (now - self.last_status_ts).nanoseconds * 1e-9
            if 0.0 <= age <= self.status_timeout:
                odom_vx = self.feedback_vx
                odom_wz = self.feedback_wz

        odom_vx = self.apply_deadzone(odom_vx, self.odom_linear_deadzone)
        odom_wz = self.apply_deadzone(odom_wz, self.odom_angular_deadzone)

        if self.feedback_yaw_available and self.applied_status_revision != self.status_revision:
            self.yaw = self.feedback_yaw
            self.applied_status_revision = self.status_revision

        yaw_mid = self.yaw + 0.5 * odom_wz * dt
        self.x += odom_vx * math.cos(yaw_mid) * dt
        self.y += odom_vx * math.sin(yaw_mid) * dt
        self.yaw = _wrap_angle(self.yaw + odom_wz * dt)

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

        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.1

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

        yaw = self.feedback_yaw if self.feedback_imu_valid else self.yaw
        imu.orientation.z = math.sin(yaw * 0.5)
        imu.orientation.w = math.cos(yaw * 0.5)
        imu.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 0.05 if self.feedback_imu_valid else 1e3,
        ]

        imu.angular_velocity.z = self.feedback_gz
        imu.angular_velocity_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 0.02 if self.feedback_imu_valid else 1e3,
        ]

        ax, ay, az = self.feedback_raw_accel
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.linear_acceleration_covariance = [
            0.3 if self.feedback_imu_accel_valid else 1e3, 0.0, 0.0,
            0.0, 0.3 if self.feedback_imu_accel_valid else 1e3, 0.0,
            0.0, 0.0, 0.3 if self.feedback_imu_accel_valid else 1e3,
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
