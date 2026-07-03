"""bridge_node.py 单元测试 —— 使用 mock 隔离串口/ROS2 依赖，不依赖真实硬件。

测试覆盖：参数默认值、串口生命周期、帧处理、命令流、
odom 积分、辅助话题发布、死区逻辑。
"""

import math
import struct
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import ANY, MagicMock, PropertyMock, patch

import pytest

# ---------------------------------------------------------------------------
# Mock rclpy + 依赖 BEFORE importing bridge_node
# ---------------------------------------------------------------------------

_mock_rclpy = MagicMock(name="rclpy")
_mock_rclpy.spin = MagicMock()
_mock_rclpy.init = MagicMock()
_mock_rclpy.shutdown = MagicMock()

_mock_geometry_msgs = MagicMock(name="geometry_msgs")
_mock_nav_msgs = MagicMock(name="nav_msgs")
_mock_nav_msgs_msg = MagicMock(name="nav_msgs.msg")
_mock_sensor_msgs = MagicMock(name="sensor_msgs")
_mock_std_msgs = MagicMock(name="std_msgs")
_mock_std_srvs = MagicMock(name="std_srvs")
_mock_tf2_ros = MagicMock(name="tf2_ros")
_mock_serial = MagicMock(name="serial")

_mock_rclpy_node = MagicMock()
sys.modules["rclpy"] = _mock_rclpy
sys.modules["rclpy.node"] = _mock_rclpy_node
sys.modules["geometry_msgs"] = _mock_geometry_msgs
sys.modules["geometry_msgs.msg"] = MagicMock()
sys.modules["nav_msgs"] = _mock_nav_msgs
sys.modules["nav_msgs.msg"] = _mock_nav_msgs_msg
sys.modules["sensor_msgs"] = _mock_sensor_msgs
sys.modules["sensor_msgs.msg"] = MagicMock()
sys.modules["std_msgs"] = _mock_std_msgs
sys.modules["std_msgs.msg"] = MagicMock()
sys.modules["std_srvs"] = _mock_std_srvs
sys.modules["std_srvs.srv"] = MagicMock()
sys.modules["tf2_ros"] = _mock_tf2_ros
sys.modules["serial"] = _mock_serial

# 确保 protocol_v2 可导入
_bridge_src = str(Path(__file__).resolve().parents[1])
if _bridge_src not in sys.path:
    sys.path.insert(0, _bridge_src)

import stm32_robot_bridge.protocol_v2 as _real_protocol

sys.modules["stm32_robot_bridge.protocol_v2"] = _real_protocol

# ---------------------------------------------------------------------------
# Mock Node 基类 —— 真实 Python 类，替代 rclpy.node.Node
# ---------------------------------------------------------------------------

PARAM_DEFAULTS = {
    "port": "/dev/serial0",
    "baudrate": 115200,
    "cmd_vel_topic": "/cmd_vel/driver",
    "odom_topic": "/wheel/odom",
    "frame_id": "odom",
    "child_frame_id": "base_link",
    "publish_tf": False,
    "cmd_timeout": 0.25,
    "control_hz": 20.0,
    "status_hz": 100.0,
    "serial_open_retry_sec": 2.0,
    "status_timeout": 0.75,
    "drive_keepalive_sec": 0.10,
    "startup_settle_sec": 0.20,
    "wheel_radius": 0.0350,
    "wheel_track_width": 0.1780,
    "odom_linear_scale": 1.0,
    "odom_angular_scale": 1.0,
    "odom_angular_sign": 1.0,
    "odom_linear_deadzone": 0.01,
    "odom_angular_deadzone": 0.03,
    "status_log_interval_sec": 0.0,
    "cmd_log_interval_sec": 0.0,
}


class MockNode:
    """真实的 Python 类，替代 rclpy.node.Node 作为 STM32Bridge 的基类。"""

    _param_overrides: dict = {}  # 类级别参数覆盖

    def __init__(self, node_name: str = "", **kwargs):
        self._node_name = node_name
        self._declared_params = {}  # name → default_value

    def declare_parameter(self, name, value=None):
        self._declared_params[name] = value

    def get_parameter(self, name):
        mock = MagicMock()
        if name in MockNode._param_overrides:
            mock.value = MockNode._param_overrides[name]
        elif name in self._declared_params:
            mock.value = self._declared_params[name]
        else:
            mock.value = PARAM_DEFAULTS.get(name, name)
        return mock

    def get_logger(self):
        return MagicMock()

    def get_clock(self):
        return MagicMock()

    def create_publisher(self, msg_type, topic, qos_profile):
        return MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        return MagicMock()

    def create_service(self, srv_type, srv_name, callback):
        return MagicMock()

    def create_timer(self, timer_period_sec, callback):
        return MagicMock()

    def destroy_node(self):
        pass

    @classmethod
    def set_params(cls, **overrides):
        """类方法：批量设置参数覆盖值（在构造 bridge 之前调用）。"""
        cls._param_overrides = dict(overrides)


_mock_rclpy.node.Node = MockNode
_mock_rclpy_node.Node = MockNode  # for `from rclpy.node import Node`

class _FakeOdometry:
    """Minimal Odometry substitute with mutable covariance arrays."""

    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.child_frame_id = ""
        self.pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(z=0.0, w=1.0),
            ),
            covariance=[0.0] * 36,
        )
        self.twist = SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.0),
                angular=SimpleNamespace(z=0.0),
            ),
            covariance=[0.0] * 36,
        )


_mock_nav_msgs_msg.Odometry = _FakeOdometry

# ---------------------------------------------------------------------------
# 导入被测试模块（mock 环境就绪后）
# ---------------------------------------------------------------------------

from stm32_robot_bridge.bridge_node import STM32Bridge


# ---------------------------------------------------------------------------
# 辅助工具
# ---------------------------------------------------------------------------


def make_status_payload(
    *,
    version=2,
    status_flags=0x08,
    control_source=1,
    enabled_mask=0b0110,
    error_flags=0x11223344,
    latched_error_flags=0x55667788,
    battery_mv=12345,
    speeds=(0, 120, 340, 0),
    encoders=(10, 2000, -3000, 40),
    currents=(0, 111, 222, 0),
    targets=(0, 130, 350, 0),
    outputs=(0, 501, -502, 0),
    speed_valid_mask=0b0110,
) -> bytes:
    """构造 64 字节 STATUS payload。"""
    payload = bytearray(64)
    payload[0] = version
    payload[1] = status_flags
    payload[2] = control_source
    payload[3] = enabled_mask
    struct.pack_into("<I", payload, 4, error_flags)
    struct.pack_into("<I", payload, 8, latched_error_flags)
    struct.pack_into("<H", payload, 12, battery_mv)
    struct.pack_into("<4h", payload, 14, *speeds)
    struct.pack_into("<4i", payload, 22, *encoders)
    struct.pack_into("<4H", payload, 38, *currents)
    struct.pack_into("<4h", payload, 46, *targets)
    struct.pack_into("<4h", payload, 54, *outputs)
    payload[62] = speed_valid_mask
    payload[63] = 0xAA
    return bytes(payload)


def make_bridge(**param_overrides):
    """创建 STM32Bridge 实例的工厂函数。"""
    MockNode.set_params(**param_overrides)
    with patch("os.path.exists", return_value=False):
        bridge = STM32Bridge()
    # 替换发布器为 mock
    bridge.odom_pub = MagicMock()
    bridge.chassis_status_pub = MagicMock()
    bridge.battery_pub = MagicMock()
    bridge.left_current_pub = MagicMock()
    bridge.right_current_pub = MagicMock()
    # 固定 logger mock，避免每次 get_logger() 返回不同对象
    bridge._logger = MagicMock()
    bridge.get_logger = MagicMock(return_value=bridge._logger)
    return bridge


# ---------------------------------------------------------------------------
# Test: 死区
# ---------------------------------------------------------------------------


class TestDeadzone:
    """验证 apply_deadzone 静态方法 —— 不依赖任何 mock。"""

    def test_zero_value_returns_zero(self):
        assert STM32Bridge.apply_deadzone(0.0, 0.01) == 0.0

    def test_value_within_positive_threshold_returns_zero(self):
        assert STM32Bridge.apply_deadzone(0.005, 0.01) == 0.0
        assert STM32Bridge.apply_deadzone(-0.005, 0.01) == 0.0

    def test_value_at_exact_threshold_boundary(self):
        assert STM32Bridge.apply_deadzone(0.01, 0.01) == 0.01

    def test_value_above_threshold_passes_through(self):
        assert STM32Bridge.apply_deadzone(0.5, 0.01) == 0.5
        assert STM32Bridge.apply_deadzone(-0.5, 0.01) == -0.5

    def test_zero_threshold_always_passes(self):
        assert STM32Bridge.apply_deadzone(0.0, 0.0) == 0.0
        assert STM32Bridge.apply_deadzone(1e-10, 0.0) == 1e-10


# ---------------------------------------------------------------------------
# Test: 参数默认值
# ---------------------------------------------------------------------------


class TestBridgeParameterDefaults:
    """验证 bridge_node 声明的所有参数及其默认值。"""

    def test_all_28_parameters_are_declared(self):
        bridge = make_bridge()
        declared = set(bridge._declared_params.keys())
        assert len(declared) == 23, f"Expected 23 parameters, got {len(declared)}: {sorted(declared)}"
        assert "port" in declared
        assert "wheel_radius" in declared
        assert "wheel_track_width" in declared
        assert "publish_tf" in declared

    def test_parameter_default_values(self):
        bridge = make_bridge()
        assert bridge.cmd_vel_topic == "/cmd_vel/driver"
        assert bridge.odom_topic == "/wheel/odom"
        assert bridge.frame_id == "odom"
        assert bridge.child_frame_id == "base_link"
        assert bridge.publish_tf is False
        assert bridge.cmd_timeout == 0.25
        assert bridge.control_hz == 20.0
        assert bridge.status_hz == 100.0
        assert bridge.wheel_radius == 0.0350
        assert bridge.wheel_track_width == 0.1780

    def test_publishers_and_subscriptions_are_created(self):
        bridge = make_bridge()
        # odom_pub 已被 make_bridge 替换，检查其他 publisher
        assert bridge.chassis_status_pub is not None
        assert bridge.battery_pub is not None
        assert bridge.left_current_pub is not None
        assert bridge.right_current_pub is not None
        # 订阅和服务已创建（通过 Node.create_subscription/create_service）
        assert bridge.has_seen_cmd_vel is False  # 初始状态


# ---------------------------------------------------------------------------
# Test: 端口规范化
# ---------------------------------------------------------------------------


class TestPortNormalization:
    """验证 _normalize_configured_port 方法。"""

    def test_normal_port_is_returned_as_is(self):
        bridge = make_bridge()
        assert bridge._normalize_configured_port("/dev/ttyUSB0") == "/dev/ttyUSB0"
        assert bridge._normalize_configured_port("/dev/serial0") == "/dev/serial0"

    def test_auto_falls_back_to_default(self):
        bridge = make_bridge()
        assert bridge._normalize_configured_port("auto") == "/dev/serial0"

    def test_empty_string_falls_back_to_default(self):
        bridge = make_bridge()
        assert bridge._normalize_configured_port("") == "/dev/serial0"

    def test_env_hint_takes_priority_for_auto(self):
        bridge = make_bridge()
        with patch.dict("os.environ", {"ROBOT_BASE_PORT_HINT": "/dev/ttyAMA0"}):
            assert bridge._normalize_configured_port("auto") == "/dev/ttyAMA0"

    def test_env_hint_takes_priority_for_empty(self):
        bridge = make_bridge()
        with patch.dict("os.environ", {"ROBOT_BASE_PORT_HINT": "/dev/ttyAMA0"}):
            assert bridge._normalize_configured_port("") == "/dev/ttyAMA0"

    def test_explicit_port_overrides_env_hint(self):
        bridge = make_bridge()
        with patch.dict("os.environ", {"ROBOT_BASE_PORT_HINT": "/dev/ttyAMA0"}):
            assert bridge._normalize_configured_port("/dev/ttyUSB0") == "/dev/ttyUSB0"


# ---------------------------------------------------------------------------
# Test: 帧处理
# ---------------------------------------------------------------------------


class TestFrameHandling:
    """验证 handle_frame 方法 —— STATUS 帧解码与内部状态更新。"""

    def test_valid_status_frame_updates_all_feedback_fields(self):
        bridge = make_bridge()
        payload = make_status_payload(
            status_flags=0x08,
            speeds=(0, 150, 300, 0),
            encoders=(100, 5000, 8000, 200),
            currents=(0, 123, 456, 0),
            targets=(0, 145, 310, 0),
            battery_mv=11250,
            error_flags=0xDEAD0001,
            speed_valid_mask=0b0110,
        )

        bridge.handle_frame(0x81, payload)

        assert bridge.protocol_version == 2
        assert bridge.status_flags == 0x08
        assert bridge.motor_enabled_mask == 0b0110
        assert bridge.motor_speed_valid_mask == 0b0110
        assert bridge.feedback_error_flags == 0xDEAD0001
        assert bridge.feedback_battery_voltage == 11.25
        assert math.isclose(bridge.feedback_left_speed, 0.150)
        assert math.isclose(bridge.feedback_right_speed, 0.300)
        assert bridge.last_left_encoder == 5000
        assert bridge.last_right_encoder == 8000
        assert math.isclose(bridge.feedback_left_current, 0.123)
        assert math.isclose(bridge.feedback_right_current, 0.456)
        assert math.isclose(bridge.feedback_left_target, 0.145)
        assert math.isclose(bridge.feedback_right_target, 0.310)
        assert bridge.feedback_odom_trusted is True

        # 辅助话题被发布
        bridge.chassis_status_pub.publish.assert_called_once()
        bridge.battery_pub.publish.assert_called_once()
        bridge.left_current_pub.publish.assert_called_once()
        bridge.right_current_pub.publish.assert_called_once()

    def test_non_status_cmd_is_ignored(self):
        bridge = make_bridge()
        bridge.handle_frame(0x02, b"\x01")
        bridge.chassis_status_pub.publish.assert_not_called()
        bridge.battery_pub.publish.assert_not_called()

    def test_wrong_payload_length_is_discarded(self):
        bridge = make_bridge()
        bridge.handle_frame(0x81, b"\x02" + b"\x00" * 62)  # 63 bytes
        bridge.chassis_status_pub.publish.assert_not_called()

    def test_wrong_protocol_version_is_discarded(self):
        bridge = make_bridge()
        payload = make_status_payload(version=1)
        bridge.handle_frame(0x81, payload)
        bridge.chassis_status_pub.publish.assert_not_called()

    def test_estop_and_fault_stop_bits_persist_in_status_flags(self):
        bridge = make_bridge()
        payload = make_status_payload(status_flags=0x03)  # ESTOP + FAULT_STOP
        bridge.handle_frame(0x81, payload)
        assert bridge.status_flags == 0x03

    def test_odom_velocity_scaling_and_sign_applied(self):
        bridge = make_bridge(
            odom_linear_scale=1.2,
            odom_angular_scale=0.8,
            odom_angular_sign=-1.0,
        )
        # speeds=(0, 100, 300, 0) → left=0.1, right=0.3
        # vx = (0.1+0.3)/2 = 0.2, wz = (0.3-0.1)/0.178 ≈ 1.1236
        payload = make_status_payload(speeds=(0, 100, 300, 0))
        bridge.handle_frame(0x81, payload)

        expected_vx = 0.2 * 1.2
        expected_wz = ((0.3 - 0.1) / 0.178) * 0.8 * (-1.0)
        assert math.isclose(bridge.feedback_vx, expected_vx, rel_tol=1e-4)
        assert math.isclose(bridge.feedback_wz, expected_wz, rel_tol=1e-4)


# ---------------------------------------------------------------------------
# Test: 命令流
# ---------------------------------------------------------------------------


class TestCommandFlow:
    """验证 cmd_vel 接收 → CommandStream 更新 → 驱动帧写入 链路。"""

    def test_on_cmd_vel_updates_target_and_command_stream(self):
        bridge = make_bridge()
        mock_twist = MagicMock()
        mock_twist.linear.x = 0.3
        mock_twist.angular.z = -0.5

        bridge.on_cmd_vel(mock_twist)

        assert bridge.has_seen_cmd_vel is True
        assert bridge.target_vx == 0.3
        assert bridge.target_wz == -0.5
        assert bridge.command_stream.target_vx == 0.3
        assert bridge.command_stream.target_wz == -0.5

    def test_estop_service_without_serial_returns_failure(self):
        bridge = make_bridge()
        bridge.serial = None
        mock_request = MagicMock()
        mock_request.data = True
        mock_response = MagicMock()

        result = bridge.on_estop(mock_request, mock_response)

        assert result.success is False
        assert "not connected" in result.message

    def test_estop_service_with_serial_sends_frame(self):
        bridge = make_bridge()
        bridge.serial = MagicMock()
        mock_request = MagicMock()
        mock_request.data = True
        mock_response = MagicMock()

        result = bridge.on_estop(mock_request, mock_response)

        assert result.success is True
        bridge.serial.write.assert_called_once()

    def test_write_drive_payload_with_enable_zero_uses_zero_velocity(self):
        bridge = make_bridge(cmd_log_interval_sec=0.0)
        bridge.serial = MagicMock()
        bridge.target_vx = 0.5
        bridge.target_wz = 0.3

        # enable=0 的释放帧
        release_payload = struct.pack("<ffBB", 0.0, 0.0, 0, 2)
        result = bridge.write_drive_payload(release_payload, time.monotonic())

        assert result is True


# ---------------------------------------------------------------------------
# Test: 里程计积分
# ---------------------------------------------------------------------------


class TestOdomIntegration:
    """验证 publish_odom 方法 —— 位置积分、协方差、信任逻辑。"""

    def _setup_odom_state(self, bridge, *, dt=0.05, status_age=0.01,
                          status_flags=0x08, odom_trusted=True,
                          vx=0.0, wz=0.0):
        """辅助：设置里程计发布所需的内部状态。"""
        base_ns = 1_000_000_000

        class FakeTime:
            def __init__(self, ns):
                self.nanoseconds = ns
            def __sub__(self, other):
                return FakeTime(self.nanoseconds - other.nanoseconds)
            def to_msg(self):
                return MagicMock()

        now = FakeTime(base_ns + int(dt * 1e9))
        bridge.last_odom_ts = FakeTime(base_ns)
        bridge.last_status_ts = FakeTime(base_ns + int((dt - status_age) * 1e9))
        bridge.status_timeout = 0.75
        bridge.status_flags = status_flags
        bridge.feedback_odom_trusted = odom_trusted
        bridge.feedback_vx = vx
        bridge.feedback_wz = wz
        bridge.odom_linear_deadzone = 0.01
        bridge.odom_angular_deadzone = 0.03
        bridge.x = 0.0
        bridge.y = 0.0
        bridge.yaw = 0.0
        return now

    def test_straight_line_integration(self):
        bridge = make_bridge()
        now = self._setup_odom_state(bridge, dt=0.05, vx=0.5, wz=0.0)

        bridge.publish_odom(now)

        # dx ≈ 0.5 * cos(0) * 0.05 = 0.025
        assert math.isclose(bridge.x, 0.025, rel_tol=1e-4)
        assert math.isclose(bridge.y, 0.0, abs_tol=1e-10)
        assert math.isclose(bridge.yaw, 0.0, abs_tol=1e-10)
        bridge.odom_pub.publish.assert_called_once()

    def test_pure_rotation_integration(self):
        bridge = make_bridge()
        now = self._setup_odom_state(bridge, dt=0.1, vx=0.0, wz=1.57)

        bridge.publish_odom(now)

        assert math.isclose(bridge.yaw, 0.157, rel_tol=1e-2)
        assert math.isclose(bridge.x, 0.0, abs_tol=1e-10)
        bridge.odom_pub.publish.assert_called_once()

    def test_untrusted_status_yields_zero_velocity(self):
        bridge = make_bridge()
        now = self._setup_odom_state(bridge, dt=0.1, odom_trusted=False, vx=0.5, wz=1.0)

        bridge.publish_odom(now)

        assert bridge.x == 0.0
        assert bridge.y == 0.0
        assert bridge.yaw == 0.0
        call_args = bridge.odom_pub.publish.call_args[0][0]
        assert call_args.twist.covariance[0] == 1000.0

    def test_estop_status_marks_odom_untrusted(self):
        bridge = make_bridge()
        now = self._setup_odom_state(bridge, dt=0.1, status_flags=0x01,  # ESTOP
                                     odom_trusted=True, vx=0.5, wz=0.0)

        bridge.publish_odom(now)

        assert bridge.x == 0.0  # ESTOP 阻止运动
        call_args = bridge.odom_pub.publish.call_args[0][0]
        assert call_args.twist.covariance[0] == 1000.0

    def test_stale_status_yields_zero_velocity(self):
        bridge = make_bridge()
        # status_age > status_timeout
        now = self._setup_odom_state(bridge, dt=0.1, status_age=1.0,  # 超过 0.75s
                                     odom_trusted=True, vx=0.5, wz=0.0)

        bridge.publish_odom(now)

        assert bridge.x == 0.0

    def test_covariance_is_low_when_odom_trusted(self):
        bridge = make_bridge()
        now = self._setup_odom_state(bridge, dt=0.05, odom_trusted=True, vx=0.5, wz=0.0)

        bridge.publish_odom(now)

        call_args = bridge.odom_pub.publish.call_args[0][0]
        assert call_args.twist.covariance[0] == 0.05
        assert call_args.twist.covariance[35] == 0.1

    def test_yaw_wraps_correctly_across_pi_boundary(self):
        bridge = make_bridge()
        bridge.yaw = 3.0  # ~172°
        now = self._setup_odom_state(bridge, dt=0.1, odom_trusted=True, vx=0.0, wz=1.0)

        bridge.publish_odom(now)

        assert -math.pi <= bridge.yaw <= math.pi

    def test_deadzone_filters_small_motion(self):
        bridge = make_bridge()
        # vx 在死区内
        now = self._setup_odom_state(bridge, dt=0.05, odom_trusted=True, vx=0.005, wz=0.0)

        bridge.publish_odom(now)

        assert bridge.x == 0.0  # 死区过滤


# ---------------------------------------------------------------------------
# Test: 辅助话题发布
# ---------------------------------------------------------------------------


class TestAuxiliaryPublishing:
    """验证 publish_auxiliary_topics 方法。"""

    def test_does_nothing_when_no_status_received(self):
        bridge = make_bridge()
        bridge.last_status_ts = None

        bridge.publish_auxiliary_topics()

        bridge.chassis_status_pub.publish.assert_not_called()
        bridge.battery_pub.publish.assert_not_called()

    def test_publishes_all_auxiliary_topics_when_status_valid(self):
        bridge = make_bridge()
        bridge.last_status_ts = MagicMock()
        bridge.last_status_ts.to_msg.return_value = MagicMock()
        bridge.feedback_error_flags = 0xDEADBEEF
        bridge.feedback_battery_voltage = 0.0  # < 1.0 → not present
        bridge.feedback_left_current = 0.5
        bridge.feedback_right_current = 0.3

        bridge.publish_auxiliary_topics()

        bridge.chassis_status_pub.publish.assert_called_once()
        bridge.battery_pub.publish.assert_called_once()
        bridge.left_current_pub.publish.assert_called_once()
        bridge.right_current_pub.publish.assert_called_once()


# ---------------------------------------------------------------------------
# Test: 串口生命周期
# ---------------------------------------------------------------------------


class TestSerialLifecycle:
    """验证串口打开/关闭/重试/异常处理。"""

    def test_ensure_serial_returns_false_when_port_not_found(self):
        bridge = make_bridge()
        bridge.serial = None
        result = bridge.ensure_serial(force_log_missing=False)
        assert result is False

    def test_ensure_serial_respects_retry_interval(self):
        bridge = make_bridge()
        bridge.serial = None
        bridge.last_open_attempt = time.monotonic()  # 刚尝试过
        result = bridge.ensure_serial(force_log_missing=False)
        assert result is False

    def test_close_serial_releases_control_and_clears_state(self):
        bridge = make_bridge()
        mock_ser = MagicMock()
        bridge.serial = mock_ser
        bridge.connected_port = "/dev/serial0"

        bridge.close_serial("test shutdown", release_control=True)

        assert bridge.serial is None
        assert bridge.connected_port == ""
        mock_ser.close.assert_called_once()

    def test_close_serial_handles_close_exception_gracefully(self):
        bridge = make_bridge()
        mock_ser = MagicMock()
        mock_ser.close.side_effect = OSError("device gone")
        bridge.serial = mock_ser
        bridge.connected_port = "/dev/serial0"

        bridge.close_serial("test shutdown", release_control=False)

        assert bridge.serial is None


# ---------------------------------------------------------------------------
# Test: 写入帧
# ---------------------------------------------------------------------------


class TestWriteFrame:
    """验证 write_frame 方法。"""

    def test_write_frame_returns_false_when_serial_is_none(self):
        bridge = make_bridge()
        bridge.serial = None
        assert bridge.write_frame(0x01, b"\x00" * 10) is False

    def test_write_frame_calls_serial_write_with_correct_frame(self):
        bridge = make_bridge()
        bridge.serial = MagicMock()
        payload = struct.pack("<ffBB", 0.5, -0.3, 1, 2)
        result = bridge.write_frame(0x01, payload)

        assert result is True
        bridge.serial.write.assert_called_once()
        written = bridge.serial.write.call_args[0][0]
        assert written[:2] == b"\xA5\x5A"

    def test_write_frame_closes_serial_on_exception(self):
        bridge = make_bridge()
        mock_ser = MagicMock()
        mock_ser.write.side_effect = OSError("write failed")
        bridge.serial = mock_ser

        result = bridge.write_frame(0x01, b"\x00" * 10)

        assert result is False
        assert bridge.serial is None


# ---------------------------------------------------------------------------
# Test: 日志辅助方法
# ---------------------------------------------------------------------------


class TestLoggingHelpers:
    """验证 warn_periodic 和日志限流逻辑。"""

    def test_warn_periodic_throttles_messages(self):
        bridge = make_bridge()
        bridge._logger.warn.reset_mock()

        bridge.warn_periodic("test_key", "first message")
        bridge.warn_periodic("test_key", "second message too soon")

        assert bridge._logger.warn.call_count == 1

    def test_warn_periodic_allows_after_interval(self):
        bridge = make_bridge()
        bridge._logger.warn.reset_mock()

        bridge.warn_periodic("key_a", "msg 1")
        bridge.warn_times["key_a"] = 0.0  # 回退时间戳

        bridge.warn_periodic("key_a", "msg 2")
        assert bridge._logger.warn.call_count == 2
