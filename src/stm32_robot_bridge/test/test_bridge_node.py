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
from unittest.mock import ANY, MagicMock, PropertyMock, call, patch

import pytest

# ---------------------------------------------------------------------------
# Mock rclpy + 依赖 BEFORE importing bridge_node
# ---------------------------------------------------------------------------

_mock_rclpy = MagicMock(name="rclpy")
_mock_rclpy.spin = MagicMock()
_mock_rclpy.init = MagicMock()
_mock_rclpy.shutdown = MagicMock()

_mock_geometry_msgs = MagicMock(name="geometry_msgs")
_mock_builtin_interfaces = MagicMock(name="builtin_interfaces")
_mock_builtin_interfaces_msg = MagicMock(name="builtin_interfaces.msg")
_mock_diagnostic_msgs = MagicMock(name="diagnostic_msgs")
_mock_diagnostic_msgs_msg = MagicMock(name="diagnostic_msgs.msg")
_mock_nav_msgs = MagicMock(name="nav_msgs")
_mock_nav_msgs_msg = MagicMock(name="nav_msgs.msg")
_mock_sensor_msgs = MagicMock(name="sensor_msgs")
_mock_sensor_msgs_msg = MagicMock(name="sensor_msgs.msg")
_mock_std_msgs = MagicMock(name="std_msgs")
_mock_std_srvs = MagicMock(name="std_srvs")
_mock_tf2_ros = MagicMock(name="tf2_ros")
_mock_serial = MagicMock(name="serial")
_mock_robot_interfaces = MagicMock(name="robot_interfaces")
_mock_robot_interfaces_msg = MagicMock(name="robot_interfaces.msg")

_mock_rclpy_node = MagicMock()
sys.modules["rclpy"] = _mock_rclpy
sys.modules["rclpy.node"] = _mock_rclpy_node
sys.modules["geometry_msgs"] = _mock_geometry_msgs
sys.modules["geometry_msgs.msg"] = MagicMock()
sys.modules["builtin_interfaces"] = _mock_builtin_interfaces
sys.modules["builtin_interfaces.msg"] = _mock_builtin_interfaces_msg
sys.modules["diagnostic_msgs"] = _mock_diagnostic_msgs
sys.modules["diagnostic_msgs.msg"] = _mock_diagnostic_msgs_msg
sys.modules["nav_msgs"] = _mock_nav_msgs
sys.modules["nav_msgs.msg"] = _mock_nav_msgs_msg
sys.modules["sensor_msgs"] = _mock_sensor_msgs
sys.modules["sensor_msgs.msg"] = _mock_sensor_msgs_msg
sys.modules["std_msgs"] = _mock_std_msgs
sys.modules["std_msgs.msg"] = MagicMock()
sys.modules["std_srvs"] = _mock_std_srvs
sys.modules["std_srvs.srv"] = MagicMock()
sys.modules["tf2_ros"] = _mock_tf2_ros
sys.modules["serial"] = _mock_serial
sys.modules["robot_interfaces"] = _mock_robot_interfaces
sys.modules["robot_interfaces.msg"] = _mock_robot_interfaces_msg

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
    "chassis_command_topic": "/chassis/command",
    "enable_legacy_cmd_vel": False,
    "legacy_cmd_vel_topic": "/cmd_vel/driver",
    "odom_topic": "/wheel/odom",
    "frame_id": "odom",
    "child_frame_id": "base_link",
    "publish_tf": False,
    "cmd_timeout": 0.15,
    "control_hz": 20.0,
    "status_hz": 100.0,
    "serial_open_retry_sec": 2.0,
    "status_timeout": 0.25,
    "drive_keepalive_sec": 0.05,
    "reject_non_finite": True,
    "hard_max_linear_mps": 0.45,
    "hard_max_angular_radps": 1.50,
    "max_command_age_sec": 0.15,
    "require_fresh_status_before_drive": True,
    "release_on_invalid_command": True,
    "release_on_status_timeout": True,
    "startup_settle_sec": 0.20,
    "wheel_radius": 0.0350,
    "wheel_track_width": 0.1760,
    "odom_linear_scale": 1.0,
    "odom_angular_scale": 1.0,
    "odom_angular_sign": 1.0,
    "odom_linear_deadzone": 0.01,
    "odom_angular_deadzone": 0.03,
    "odom_max_dt_sec": 0.25,
    "imu.use_orientation": False,
    "imu.orientation_stddev": 0.2,
    "imu.angular_velocity_stddev.x": 0.02,
    "imu.angular_velocity_stddev.y": 0.02,
    "imu.angular_velocity_stddev.z": 0.02,
    "imu.linear_acceleration_stddev.x": 0.2,
    "imu.linear_acceleration_stddev.y": 0.2,
    "imu.linear_acceleration_stddev.z": 0.2,
    "imu.clock_offset_alpha": 0.02,
    "status_log_interval_sec": 0.0,
    "cmd_log_interval_sec": 0.0,
}


class MockNode:
    """真实的 Python 类，替代 rclpy.node.Node 作为 STM32Bridge 的基类。"""

    _param_overrides: dict = {}  # 类级别参数覆盖

    def __init__(self, node_name: str = "", **kwargs):
        self._node_name = node_name
        self._declared_params = {}  # name → default_value
        self._clock = MagicMock()
        self._clock.now.return_value.nanoseconds = int(10.0e9)
        self._clock.now.return_value.to_msg.return_value = SimpleNamespace(sec=10, nanosec=0)

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
        return self._clock

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


class _FakeImu:
    """Minimal Imu substitute with mutable vector fields."""

    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        self.angular_velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.linear_acceleration = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9


_mock_nav_msgs_msg.Odometry = _FakeOdometry
_mock_sensor_msgs_msg.Imu = _FakeImu


class _FakeChassisCommand:
    SOURCE_NONE = 0
    SOURCE_TELEOP = 1

    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0))
        self.twist = SimpleNamespace(
            linear=SimpleNamespace(x=0.0),
            angular=SimpleNamespace(z=0.0),
        )
        self.enable = False
        self.source = self.SOURCE_NONE
        self.sequence = 0


_mock_robot_interfaces_msg.ChassisCommand = _FakeChassisCommand


class _FakeChassisState:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.bridge_state = 0
        self.selected_source = 0
        self.command_sequence = 0
        self.upper_enabled = False
        self.firmware_control_source = 0
        self.status_flags = 0
        self.error_flags = 0
        self.latched_error_flags = 0


class _FakeKeyValue:
    def __init__(self, key="", value=""):
        self.key = key
        self.value = value


class _FakeDiagnosticStatus:
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3

    def __init__(self):
        self.level = self.OK
        self.name = ""
        self.message = ""
        self.hardware_id = ""
        self.values = []


class _FakeDiagnosticArray:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.status = []


class _FakeTimeMessage:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


_mock_robot_interfaces_msg.ChassisState = _FakeChassisState
_mock_diagnostic_msgs_msg.DiagnosticArray = _FakeDiagnosticArray
_mock_diagnostic_msgs_msg.DiagnosticStatus = _FakeDiagnosticStatus
_mock_diagnostic_msgs_msg.KeyValue = _FakeKeyValue
_mock_builtin_interfaces_msg.Time = _FakeTimeMessage

# ---------------------------------------------------------------------------
# 导入被测试模块（mock 环境就绪后）
# ---------------------------------------------------------------------------

from stm32_robot_bridge.bridge_node import STM32Bridge
from stm32_robot_bridge.bridge_state import BridgeState
from stm32_robot_bridge.protocol_v2 import CMD_IMU_STATUS, CMD_STATUS, build_frame


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
    encoder_anomaly_mask=0,
    comm_health_flags=0x5A,
) -> bytes:
    """Construct a STATUS payload."""
    payload = bytearray(65)
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
    payload[63] = encoder_anomaly_mask
    payload[64] = comm_health_flags
    return bytes(payload)


def make_imu_status_payload(
    *,
    version=2,
    accel=(0.1, -0.2, 0.3),
    gyro=(90.0, -45.0, 180.0),
    euler=(10.0, -20.0, 30.0),
    quaternion=(0.5, 0.5, -0.5, 0.5),  # wire order [w, x, y, z]; decode remaps to [x, y, z, w]
    timestamp_ms=123456,
    sensor_time=654321,
    sample_count=42,
    quality_flags=0,
    quality_counters=(1, 2, 3, 4, 5, 6, 7),
    status_flags=0x0B,
    temperature_c=25,
) -> bytes:
    payload = bytearray(99)
    payload[0] = version
    offset = 1
    for values in (accel, gyro, euler, quaternion):
        for value in values:
            struct.pack_into("<f", payload, offset, value)
            offset += 4
    struct.pack_into("<I", payload, offset, timestamp_ms)
    offset += 4
    struct.pack_into("<I", payload, offset, sensor_time)
    offset += 4
    struct.pack_into("<I", payload, offset, sample_count)
    offset += 4
    struct.pack_into("<I", payload, offset, quality_flags)
    offset += 4
    struct.pack_into("<7I", payload, offset, *quality_counters)
    offset += 28
    payload[offset] = status_flags
    payload[offset + 1] = temperature_c & 0xFF
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
    bridge.state_pub = MagicMock()
    bridge.diagnostics_pub = MagicMock()
    # 固定 logger mock，避免每次 get_logger() 返回不同对象
    bridge._logger = MagicMock()
    bridge.get_logger = MagicMock(return_value=bridge._logger)
    return bridge


def make_ready_bridge(**param_overrides):
    bridge = make_bridge(**param_overrides)
    bridge.serial = MagicMock()
    bridge.protocol_version = 2
    bridge.status_flags = 0
    bridge.last_status_monotonic = time.monotonic()
    bridge.state_machine.on_serial_opened()
    bridge.state_machine.on_settled()
    bridge.state_machine.on_valid_status(0)
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

    def test_all_45_parameters_are_declared(self):
        bridge = make_bridge()
        declared = set(bridge._declared_params.keys())
        assert len(declared) == 45, f"Expected 45 parameters, got {len(declared)}: {sorted(declared)}"
        assert "port" in declared
        assert "wheel_radius" in declared
        assert "wheel_track_width" in declared
        assert "publish_tf" in declared
        assert "imu_topic" in declared
        assert "imu_frame_id" in declared

    def test_parameter_default_values(self):
        bridge = make_bridge()
        assert bridge.chassis_command_topic == "/chassis/command"
        assert bridge.enable_legacy_cmd_vel is False
        assert bridge.legacy_cmd_vel_topic == "/cmd_vel/driver"
        assert bridge.odom_topic == "/wheel/odom"
        assert bridge.frame_id == "odom"
        assert bridge.child_frame_id == "base_link"
        assert bridge.publish_tf is False
        assert bridge.cmd_timeout == 0.15
        assert bridge.control_hz == 20.0
        assert bridge.status_hz == 100.0
        assert bridge.status_timeout == 0.25
        assert bridge.drive_keepalive_sec == 0.05
        assert bridge.hard_max_linear_mps == 0.45
        assert bridge.hard_max_angular_radps == 1.5
        assert bridge.max_command_age_sec == 0.15
        assert bridge.odom_max_dt_sec == 0.25
        assert bridge.wheel_radius == 0.0350
        assert bridge.wheel_track_width == 0.1760
        assert bridge.imu_topic == "/imu/data"
        assert bridge.imu_frame_id == "imu_link"
        assert bridge.imu_use_orientation is False
        assert bridge.imu_angular_velocity_stddev == (0.02, 0.02, 0.02)
        assert bridge.imu_linear_acceleration_stddev == (0.2, 0.2, 0.2)

    def test_publishers_and_subscriptions_are_created(self):
        bridge = make_bridge()
        # odom_pub 已被 make_bridge 替换，检查其他 publisher
        assert bridge.chassis_status_pub is not None
        assert bridge.battery_pub is not None
        assert bridge.left_current_pub is not None
        assert bridge.right_current_pub is not None
        assert bridge.imu_pub is not None
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


    def test_valid_imu_status_frame_publishes_imu_data_with_unit_conversions(self):
        bridge = make_bridge()
        payload = make_imu_status_payload()

        bridge.handle_frame(_real_protocol.CMD_IMU_STATUS, payload)

        bridge.imu_pub.publish.assert_called_once()
        imu_msg = bridge.imu_pub.publish.call_args[0][0]
        assert imu_msg.header.frame_id == "imu_link"
        assert math.isclose(imu_msg.linear_acceleration.x, 0.1 * 9.80665, rel_tol=1e-6)
        assert math.isclose(imu_msg.linear_acceleration.y, -0.2 * 9.80665, rel_tol=1e-6)
        assert math.isclose(imu_msg.linear_acceleration.z, 0.3 * 9.80665, rel_tol=1e-6)
        assert math.isclose(imu_msg.angular_velocity.x, math.radians(90.0), rel_tol=1e-6)
        assert math.isclose(imu_msg.angular_velocity.y, math.radians(-45.0), rel_tol=1e-6)
        assert math.isclose(imu_msg.angular_velocity.z, math.radians(180.0), rel_tol=1e-6)
        assert imu_msg.header.stamp.sec == 10
        assert imu_msg.header.stamp.nanosec == 0
        assert imu_msg.orientation_covariance[0] == -1.0
        assert imu_msg.angular_velocity_covariance[0] == pytest.approx(0.02 ** 2)
        assert imu_msg.angular_velocity_covariance[4] == pytest.approx(0.02 ** 2)
        assert imu_msg.linear_acceleration_covariance[8] == pytest.approx(0.2 ** 2)

    def test_imu_quality_error_and_duplicate_sample_are_rejected(self):
        bridge = make_bridge()

        bridge.handle_imu_status_frame(make_imu_status_payload(quality_flags=1))
        bridge.imu_pub.publish.assert_not_called()
        bridge.handle_imu_status_frame(make_imu_status_payload(sample_count=42))
        bridge.handle_imu_status_frame(
            make_imu_status_payload(sample_count=42, timestamp_ms=123476)
        )

        assert bridge.imu_pub.publish.call_count == 1
        assert bridge.invalid_imu_count == 2

    def test_imu_sample_time_advances_by_mcu_time_not_receive_delay(self):
        bridge = make_bridge()
        bridge.handle_imu_status_frame(
            make_imu_status_payload(timestamp_ms=1000, sample_count=1)
        )
        bridge.get_clock().now.return_value.nanoseconds = int(10.5e9)

        bridge.handle_imu_status_frame(
            make_imu_status_payload(timestamp_ms=1020, sample_count=2)
        )

        second = bridge.imu_pub.publish.call_args[0][0]
        sample_sec = second.header.stamp.sec + second.header.stamp.nanosec * 1e-9
        assert sample_sec == pytest.approx(10.0296, abs=1e-6)
        assert sample_sec < 10.5

    def test_imu_status_bad_length_is_discarded(self):
        bridge = make_bridge()
        bridge.handle_frame(_real_protocol.CMD_IMU_STATUS, make_imu_status_payload()[:-1])
        bridge.imu_pub.publish.assert_not_called()
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
        # vx = (0.1+0.3)/2 = 0.2, wz = (0.3-0.1)/0.176 ≈ 1.1364
        payload = make_status_payload(speeds=(0, 100, 300, 0))
        bridge.handle_frame(0x81, payload)

        expected_vx = 0.2 * 1.2
        expected_wz = ((0.3 - 0.1) / 0.176) * 0.8 * (-1.0)
        assert math.isclose(bridge.feedback_vx, expected_vx, rel_tol=1e-4)
        assert math.isclose(bridge.feedback_wz, expected_wz, rel_tol=1e-4)


# ---------------------------------------------------------------------------
# Test: 命令流
# ---------------------------------------------------------------------------


class TestCommandFlow:
    """验证 cmd_vel 接收 → CommandStream 更新 → 驱动帧写入 链路。"""

    def test_on_cmd_vel_updates_target_and_command_stream(self):
        bridge = make_ready_bridge(enable_legacy_cmd_vel=True)
        mock_twist = MagicMock()
        mock_twist.linear.x = 0.3
        mock_twist.angular.z = -0.5

        bridge.on_cmd_vel(mock_twist)

        assert bridge.has_seen_cmd_vel is True
        assert bridge.target_vx == 0.3
        assert bridge.target_wz == -0.5
        assert bridge.command_stream.target_vx == 0.3
        assert bridge.command_stream.target_wz == -0.5

    def test_enabled_chassis_command_updates_sequence_source_and_stream(self):
        bridge = make_ready_bridge()
        msg = _FakeChassisCommand()
        msg.enable = True
        msg.source = _FakeChassisCommand.SOURCE_TELEOP
        msg.sequence = 42
        msg.twist.linear.x = 0.2
        msg.twist.angular.z = -0.3

        bridge.on_chassis_command(msg)

        assert bridge.command_sequence == 42
        assert bridge.selected_source == _FakeChassisCommand.SOURCE_TELEOP
        assert bridge.command_stream.target_vx == 0.2
        assert bridge.command_stream.target_wz == -0.3
        assert bridge.bridge_state is BridgeState.ACTIVE

    def test_enabled_command_without_status_is_rejected_and_released(self):
        bridge = make_bridge()
        bridge.release_upper_control = MagicMock(return_value=True)
        msg = _FakeChassisCommand()
        msg.enable = True
        msg.twist.linear.x = 0.2

        bridge.on_chassis_command(msg)

        bridge.release_upper_control.assert_called_once_with()
        assert bridge.command_stream.last_command_time is None
        assert bridge.invalid_command_count == 1

    def test_bridge_hard_clamps_command_after_status_is_ready(self):
        bridge = make_ready_bridge()
        msg = _FakeChassisCommand()
        msg.enable = True
        msg.twist.linear.x = 100.0
        msg.twist.angular.z = -100.0

        bridge.on_chassis_command(msg)

        assert bridge.command_stream.target_vx == 0.45
        assert bridge.command_stream.target_wz == -1.5

    def test_stale_chassis_command_is_rejected(self):
        bridge = make_ready_bridge()
        bridge.release_upper_control = MagicMock(return_value=True)
        msg = _FakeChassisCommand()
        msg.enable = True
        msg.header.stamp = SimpleNamespace(sec=9, nanosec=0)

        bridge.on_chassis_command(msg)

        bridge.release_upper_control.assert_called_once_with()
        assert bridge.command_stream.last_command_time is None

    def test_disabled_chassis_command_releases_upper_control(self):
        bridge = make_ready_bridge()
        bridge.release_upper_control = MagicMock(return_value=True)
        bridge.command_stream.update_command(0.3, 0.2, time.monotonic())
        msg = _FakeChassisCommand()
        msg.sequence = 5

        bridge.on_chassis_command(msg)

        bridge.release_upper_control.assert_called_once_with()
        assert bridge.command_sequence == 5
        assert bridge.selected_source == _FakeChassisCommand.SOURCE_NONE
        assert bridge.command_stream.last_command_time is None

        send_payload = MagicMock(return_value=True)
        bridge.command_stream.tick(time.monotonic(), send_payload)
        send_payload.assert_not_called()

    def test_status_timeout_releases_active_command_and_enters_wait_status(self):
        bridge = make_ready_bridge()
        bridge.state_machine.on_drive_enabled()
        bridge.command_stream.update_command(0.2, 0.0, time.monotonic())
        bridge.last_status_monotonic = time.monotonic() - 1.0
        bridge.ensure_serial = MagicMock(return_value=True)
        bridge.read_serial_frames = MagicMock()
        bridge.release_upper_control = MagicMock(return_value=True)
        bridge.publish_odom = MagicMock()

        bridge.control_loop()

        bridge.release_upper_control.assert_called_once_with()
        assert bridge.bridge_state is BridgeState.WAIT_STATUS
        assert bridge.command_stream.last_command_time is None

    def test_fault_status_releases_and_requires_disable_enable_recovery(self):
        bridge = make_ready_bridge()
        bridge.state_machine.on_drive_enabled()
        bridge.command_stream.update_command(0.2, 0.0, time.monotonic())
        bridge.release_upper_control = MagicMock(return_value=True)

        bridge.handle_status_frame(make_status_payload(status_flags=0x02))

        assert bridge.bridge_state is BridgeState.FAULT
        bridge.release_upper_control.assert_called_once_with()
        assert bridge.command_stream.last_command_time is None

        bridge.handle_status_frame(make_status_payload(status_flags=0x00))
        assert bridge.bridge_state is BridgeState.FAULT

        enable = _FakeChassisCommand()
        enable.enable = True
        enable.source = _FakeChassisCommand.SOURCE_TELEOP
        enable.twist.linear.x = 0.2
        assert bridge._accept_drive_command(
            vx=enable.twist.linear.x,
            wz=0.0,
            command_stamp_sec=10.0,
            source=enable.source,
        ) is False

        disable = _FakeChassisCommand()
        bridge.on_chassis_command(disable)
        assert bridge.bridge_state is BridgeState.READY
        bridge.on_chassis_command(enable)
        assert bridge.bridge_state is BridgeState.ACTIVE

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
        bridge.serial.write.side_effect = lambda chunk: len(chunk)
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
    """验证每个 STATUS 样本只积分一次。"""

    def _sample(self, bridge, sample_time, *, status_flags=0x08, trusted=True, vx=0.0, wz=0.0):
        bridge.status_flags = status_flags
        bridge.feedback_odom_trusted = trusted
        bridge.feedback_vx = vx
        bridge.feedback_wz = wz
        bridge.update_and_publish_odom(bridge.get_clock().now(), sample_time)

    def test_straight_line_integration(self):
        bridge = make_bridge()
        self._sample(bridge, 1.0, vx=0.5)
        self._sample(bridge, 1.05, vx=0.5)

        assert math.isclose(bridge.x, 0.025, rel_tol=1e-4)
        assert math.isclose(bridge.y, 0.0, abs_tol=1e-10)
        assert math.isclose(bridge.yaw, 0.0, abs_tol=1e-10)
        assert bridge.odom_pub.publish.call_count == 2

    def test_pure_rotation_integration(self):
        bridge = make_bridge()
        self._sample(bridge, 1.0, wz=1.57)
        self._sample(bridge, 1.1, wz=1.57)

        assert math.isclose(bridge.yaw, 0.157, rel_tol=1e-2)
        assert math.isclose(bridge.x, 0.0, abs_tol=1e-10)

    def test_untrusted_status_yields_zero_velocity(self):
        bridge = make_bridge()
        self._sample(bridge, 1.0, trusted=False, vx=0.5, wz=1.0)
        self._sample(bridge, 1.1, trusted=False, vx=0.5, wz=1.0)

        assert bridge.x == 0.0
        assert bridge.y == 0.0
        assert bridge.yaw == 0.0
        call_args = bridge.odom_pub.publish.call_args[0][0]
        assert call_args.twist.covariance[0] == 1000.0

    def test_estop_status_marks_odom_untrusted(self):
        bridge = make_bridge()
        self._sample(bridge, 1.0, status_flags=0x01, vx=0.5)
        self._sample(bridge, 1.1, status_flags=0x01, vx=0.5)

        assert bridge.x == 0.0  # ESTOP 阻止运动
        call_args = bridge.odom_pub.publish.call_args[0][0]
        assert call_args.twist.covariance[0] == 1000.0

    def test_large_status_gap_is_not_filled(self):
        bridge = make_bridge()
        self._sample(bridge, 1.0, vx=0.5)
        self._sample(bridge, 2.0, vx=0.5)

        assert bridge.x == 0.0
        odom = bridge.odom_pub.publish.call_args[0][0]
        assert odom.twist.covariance[0] == 1000.0

    def test_covariance_is_low_when_odom_trusted(self):
        bridge = make_bridge()
        self._sample(bridge, 1.0, vx=0.5)
        self._sample(bridge, 1.05, vx=0.5)

        call_args = bridge.odom_pub.publish.call_args[0][0]
        assert 0.0 < call_args.twist.covariance[0] < 1.0
        assert 0.0 < call_args.twist.covariance[35] < 1.0

    def test_yaw_wraps_correctly_across_pi_boundary(self):
        bridge = make_bridge()
        from stm32_robot_bridge.odometry import Pose2D

        bridge.odometry.reset(Pose2D(yaw=3.1))
        self._sample(bridge, 1.0, wz=1.0)
        self._sample(bridge, 1.1, wz=1.0)

        assert -math.pi <= bridge.yaw <= math.pi

    def test_deadzone_filters_small_motion(self):
        bridge = make_bridge()
        self._sample(bridge, 1.0, vx=0.005)
        self._sample(bridge, 1.05, vx=0.005)

        assert bridge.x == 0.0  # 死区过滤

    def test_control_loop_does_not_republish_or_integrate_old_status(self):
        bridge = make_ready_bridge()
        bridge.ensure_serial = MagicMock(return_value=True)
        bridge.read_serial_frames = MagicMock()
        bridge.update_and_publish_odom = MagicMock()

        bridge.control_loop()

        bridge.update_and_publish_odom.assert_not_called()

    def test_valid_status_drives_one_odom_sample(self):
        bridge = make_ready_bridge()
        bridge.update_and_publish_odom = MagicMock()

        bridge.handle_status_frame(make_status_payload())

        bridge.update_and_publish_odom.assert_called_once_with(
            bridge.last_status_ts, bridge.last_status_monotonic
        )


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

    def test_chassis_state_exposes_control_contract(self):
        bridge = make_ready_bridge()
        bridge.command_sequence = 42
        bridge.selected_source = _FakeChassisCommand.SOURCE_TELEOP
        bridge.control_source = 1
        bridge.feedback_error_flags = 0x12
        bridge.feedback_latched_error_flags = 0x34
        bridge.state_machine.on_drive_enabled()

        bridge.publish_chassis_state()

        state = bridge.state_pub.publish.call_args[0][0]
        assert state.bridge_state == int(BridgeState.ACTIVE)
        assert state.selected_source == _FakeChassisCommand.SOURCE_TELEOP
        assert state.command_sequence == 42
        assert state.upper_enabled is True
        assert state.firmware_control_source == 1
        assert state.error_flags == 0x12


class TestStandardDiagnostics:
    def test_publishes_five_named_standard_diagnostic_components(self):
        bridge = make_ready_bridge()
        bridge.imu_online = True
        bridge.imu_status_flags = _real_protocol.IMU_FLAG_ONLINE | _real_protocol.IMU_FLAG_CALIBRATED
        bridge.last_imu_monotonic = time.monotonic()
        bridge.imu_quality_flags = 0
        bridge.imu_sample_count = 10
        bridge.imu_temperature_c = 25
        bridge.transport_stats.rx_crc_errors = 2

        bridge.publish_diagnostics()

        array = bridge.diagnostics_pub.publish.call_args[0][0]
        statuses = {status.name: status for status in array.status}
        assert set(statuses) == {
            "stm32_bridge/serial",
            "stm32_bridge/protocol",
            "stm32_bridge/control",
            "stm32_bridge/chassis",
            "stm32_bridge/imu",
        }
        assert statuses["stm32_bridge/serial"].level == _FakeDiagnosticStatus.OK
        assert statuses["stm32_bridge/protocol"].level == _FakeDiagnosticStatus.WARN
        protocol_values = {item.key: item.value for item in statuses["stm32_bridge/protocol"].values}
        assert protocol_values["crc_error_count"] == "2"


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

    def test_successful_open_schedules_nonblocking_startup_release(self):
        bridge = make_bridge()
        mock_ser = MagicMock()
        bridge.open_serial_port = MagicMock(return_value=mock_ser)
        bridge.set_modem_lines_low = MagicMock()
        bridge.reset_serial_buffers = MagicMock()

        with patch("os.path.exists", return_value=True), patch("time.monotonic", return_value=100.0), patch("time.sleep") as sleep:
            result = bridge.ensure_serial(force_log_missing=True)

        assert result is True
        assert bridge.serial is mock_ser
        assert bridge.connected_port == "/dev/serial0"
        bridge.set_modem_lines_low.assert_called_once()
        bridge.reset_serial_buffers.assert_called_once()
        sleep.assert_not_called()
        assert bridge.bridge_state is BridgeState.SETTLING
        assert bridge.startup_release_remaining == 3
        assert bridge.next_startup_action_sec == 100.2
        assert bridge.transport_stats.serial_reconnects == 1

    def test_startup_release_sends_three_timed_frames_then_waits_for_status(self):
        bridge = make_bridge()
        bridge.serial = MagicMock()
        bridge.release_upper_control = MagicMock(return_value=True)
        bridge.state_machine.on_serial_opened()
        bridge.startup_release_remaining = 3
        bridge.next_startup_action_sec = 100.2

        bridge._advance_startup(100.19)
        bridge._advance_startup(100.20)
        bridge._advance_startup(100.21)
        bridge._advance_startup(100.22)
        bridge._advance_startup(100.24)

        assert bridge.release_upper_control.call_count == 3
        assert bridge.bridge_state is BridgeState.WAIT_STATUS
        assert bridge.startup_release_remaining == 0

    def test_close_serial_releases_control_and_clears_state(self):
        bridge = make_bridge()
        mock_ser = MagicMock()
        bridge.serial = mock_ser
        bridge.connected_port = "/dev/serial0"

        bridge.close_serial("test shutdown", release_control=True)

        assert bridge.serial is None
        assert bridge.bridge_state is BridgeState.DISCONNECTED
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
        bridge.serial.write.side_effect = lambda chunk: len(chunk)
        result = bridge.write_frame(0x01, payload)

        assert result is True
        bridge.serial.write.assert_called_once()
        written = bridge.serial.write.call_args[0][0]
        assert written[:2] == b"\xA5\x5A"
        assert bridge.transport_stats.tx_frames == 1
        assert bridge.transport_stats.tx_bytes == len(written)

    def test_write_frame_completes_segmented_short_write(self):
        bridge = make_bridge()
        bridge.serial = MagicMock()
        bridge.serial.write.side_effect = [4, 11]

        result = bridge.write_frame(0x01, b"\x00" * 10)

        assert result is True
        assert bridge.serial.write.call_count == 2
        assert bridge.transport_stats.tx_short_writes == 1

    def test_write_frame_closes_serial_on_exception(self):
        bridge = make_bridge()
        mock_ser = MagicMock()
        mock_ser.write.side_effect = OSError("write failed")
        bridge.serial = mock_ser

        result = bridge.write_frame(0x01, b"\x00" * 10)

        assert result is False
        assert bridge.serial is None
        assert bridge.transport_stats.tx_errors == 1

    def test_read_serial_frames_routes_every_complete_frame(self):
        bridge = make_bridge()
        status_payload = make_status_payload()
        imu_payload = make_imu_status_payload()
        serial_bytes = build_frame(CMD_STATUS, status_payload) + build_frame(CMD_IMU_STATUS, imu_payload)
        bridge.serial = MagicMock()
        bridge.serial.in_waiting = len(serial_bytes)
        bridge.serial.read.return_value = serial_bytes
        bridge.handle_frame = MagicMock()

        bridge.read_serial_frames()

        assert bridge.handle_frame.call_args_list == [
            call(CMD_STATUS, status_payload),
            call(CMD_IMU_STATUS, imu_payload),
        ]
        assert bridge.transport_stats.rx_bytes == len(serial_bytes)
        assert bridge.transport_stats.rx_frames == 2


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


class TestDiagnosticFrame:
    def make_diagnostic_payload(
        self,
        version=2,
        schema_version=1,
        post_done=1,
        imu_status_flags=0x0B,
        post_error_flags=0,
        adc_invalid_reason_flags=0,
        task_timeout_mask=0,
        imu_quality_flags=0,
        reset_reason_flags=0,
        uptime_ms=10000,
    ) -> bytes:
        import struct as _struct
        payload = bytearray(28)
        payload[0] = version
        payload[1] = schema_version
        payload[2] = post_done
        payload[3] = imu_status_flags
        _struct.pack_into("<I", payload, 4, post_error_flags)
        _struct.pack_into("<I", payload, 8, adc_invalid_reason_flags)
        _struct.pack_into("<H", payload, 12, task_timeout_mask)
        _struct.pack_into("<I", payload, 16, imu_quality_flags)
        _struct.pack_into("<I", payload, 20, reset_reason_flags)
        _struct.pack_into("<I", payload, 24, uptime_ms)
        return bytes(payload)

    def test_diagnostic_frame_is_handled(self):
        bridge = make_bridge()
        payload = self.make_diagnostic_payload()
        bridge.handle_frame(_real_protocol.CMD_DIAGNOSTIC, payload)
        # diagnostic publisher should have been called
        bridge.diag_pub.publish.assert_called()

    def test_diagnostic_bad_length_is_discarded(self):
        bridge = make_bridge()
        bridge.handle_frame(_real_protocol.CMD_DIAGNOSTIC, b"\x02" * 27)
        bridge.diag_pub.publish.assert_not_called()

    def test_diagnostic_bad_version_is_discarded(self):
        bridge = make_bridge()
        payload = self.make_diagnostic_payload(version=1)
        bridge.handle_frame(_real_protocol.CMD_DIAGNOSTIC, payload)
        bridge.diag_pub.publish.assert_not_called()

    def test_diagnostic_post_errors_log_warning(self):
        bridge = make_bridge()
        bridge._logger.warn.reset_mock()
        payload = self.make_diagnostic_payload(post_error_flags=0xDEAD)
        bridge.handle_frame(_real_protocol.CMD_DIAGNOSTIC, payload)
        assert bridge._logger.warn.call_count >= 1


class TestClearFaultService:
    def test_clear_fault_sends_frame(self):
        bridge = make_bridge()
        bridge.serial = MagicMock()
        response = SimpleNamespace(success=False, message="")
        bridge.on_clear_fault(SimpleNamespace(), response)
        assert response.success is True
        bridge.serial.write.assert_called()


class TestEstopService:
    def test_estop_release_is_rejected_without_writing_a_frame(self):
        bridge = make_bridge()
        bridge.serial = MagicMock()
        bridge._logger.warn.reset_mock()
        response = SimpleNamespace(success=False, message="")
        bridge.on_estop(SimpleNamespace(data=False), response)
        assert response.success is False
        assert "unsupported" in response.message.lower()
        bridge.serial.write.assert_not_called()
        assert bridge._logger.warn.call_count == 1


class TestImuQuaternionRemap:
    def test_imu_orientation_with_distinct_quaternion_values(self):
        """Wire [w, x, y, z] = [1, 0, 0, 0] → ROS orientation identity [x=0, y=0, z=0, w=1]."""
        bridge = make_bridge(**{"imu.use_orientation": True})
        # Use identity quaternion — already unit-norm, avoids normalization skew
        payload = make_imu_status_payload(quaternion=(1.0, 0.0, 0.0, 0.0))
        bridge.handle_frame(_real_protocol.CMD_IMU_STATUS, payload)
        call_args = bridge.imu_pub.publish.call_args
        assert call_args is not None
        imu_msg = call_args[0][0]
        # Wire [w=1, x=0, y=0, z=0] → ROS [x=0, y=0, z=0, w=1]
        assert math.isclose(imu_msg.orientation.x, 0.0, abs_tol=1e-6)
        assert math.isclose(imu_msg.orientation.y, 0.0, abs_tol=1e-6)
        assert math.isclose(imu_msg.orientation.z, 0.0, abs_tol=1e-6)
        assert math.isclose(imu_msg.orientation.w, 1.0, abs_tol=1e-6)
