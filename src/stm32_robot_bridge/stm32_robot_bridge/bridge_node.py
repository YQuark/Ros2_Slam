#!/usr/bin/env python3
import math
import struct
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
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
MSG_ACK = 0x80
MSG_NACK = 0x81

FLAG_ACK_REQ = 0x01

MODE_CLOSED_LOOP = 2


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

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.declare_parameter('max_linear', 0.50)
        self.declare_parameter('max_angular', 1.50)
        self.declare_parameter('cmd_timeout', 0.30)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('status_hz', 5.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.status_hz = float(self.get_parameter('status_hz').value)

        self.serial = None
        self.rx_buf = bytearray()
        self.seq = 0

        self.last_cmd_time = self.get_clock().now()
        self.target_vx = 0.0
        self.target_wz = 0.0

        self.current_vx = 0.0
        self.current_wz = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_ts = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 20)

        self.create_timer(1.0 / max(self.control_hz, 1.0), self.control_loop)
        self.create_timer(1.0 / max(self.status_hz, 1.0), self.status_poll)

        self.open_serial()
        self.send_set_mode(MODE_CLOSED_LOOP)
        self.get_logger().info(f'STM32 bridge started on {self.port}@{self.baudrate}')

    def open_serial(self) -> None:
        if serial is None:
            self.get_logger().error('python3-serial not installed')
            return
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.0)
        except Exception as e:
            self.serial = None
            self.get_logger().error(f'Failed to open serial {self.port}: {e}')

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
            self.get_logger().error(f'Serial write failed: {e}')
            return False

    def on_cmd_vel(self, msg: Twist) -> None:
        self.target_vx = float(msg.linear.x)
        self.target_wz = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self) -> None:
        self.read_serial_frames()

        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            vx = 0.0
            wz = 0.0
        else:
            vx = self.target_vx
            wz = self.target_wz

        self.current_vx = vx
        self.current_wz = wz

        # 固件协议是归一化 [-1, 1]，这里按可配置上限做线性映射。
        v_norm = max(-1.0, min(1.0, vx / max(self.max_linear, 1e-6)))
        w_norm = max(-1.0, min(1.0, wz / max(self.max_angular, 1e-6)))

        v_q15 = int(max(-32767, min(32767, round(v_norm * 32767.0))))
        w_q15 = int(max(-32767, min(32767, round(w_norm * 32767.0))))

        payload = struct.pack('<hh', v_q15, w_q15)
        self.write_frame(MSG_CMD_SET_DRIVE, payload, ack_req=True)

        self.publish_odom(now)

    def status_poll(self) -> None:
        self.read_serial_frames()
        self.write_frame(MSG_CMD_GET_STATUS, b'', ack_req=True)

    def send_set_mode(self, mode: int) -> None:
        self.write_frame(MSG_CMD_SET_MODE, bytes([mode]), ack_req=True)

    def read_serial_frames(self) -> None:
        if self.serial is None:
            return
        try:
            waiting = self.serial.in_waiting
            if waiting > 0:
                self.rx_buf.extend(self.serial.read(waiting))
        except Exception as e:
            self.get_logger().error(f'Serial read failed: {e}')
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

        if msg_type != MSG_ACK or len(payload) < 4:
            return

        if len(payload) <= 4:
            return

        ext = payload[4:]
        # GET_STATUS ACK 扩展区: tick(4) mode(1) src(1) vb_mv(2) pct_x10(2) v_q15(2) w_q15(2) gz_x10(2) imu_en(1) imu_valid(1)
        if len(ext) >= 18:
            _, mode, src, vb_mv, pct_x10, v_q15, w_q15, _, imu_en, imu_valid = struct.unpack('<IBBHHhhhBB', ext[:18])
            self.get_logger().debug(
                f'status mode={mode} src={src} vb={vb_mv/1000.0:.2f}V pct={pct_x10/10.0:.1f}% '
                f'cmd=({v_q15/32767.0:.2f},{w_q15/32767.0:.2f}) imu={imu_en}/{imu_valid}'
            )

    def publish_odom(self, now) -> None:
        dt = (now - self.last_odom_ts).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_odom_ts = now

        yaw_mid = self.yaw + 0.5 * self.current_wz * dt
        self.x += self.current_vx * math.cos(yaw_mid) * dt
        self.y += self.current_vx * math.sin(yaw_mid) * dt
        self.yaw += self.current_wz * dt

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

        odom.twist.twist.linear.x = self.current_vx
        odom.twist.twist.angular.z = self.current_wz

        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.1

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
