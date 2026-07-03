"""cmd_vel_mux 节点单元测试 —— 验证多源仲裁、优先级、超时逻辑。

使用 mock rclpy 环境，不依赖 ROS2 运行时。
"""

import sys
from unittest.mock import MagicMock, patch

import pytest

# ---------------------------------------------------------------------------
# Mock rclpy + 依赖
# ---------------------------------------------------------------------------

_mock_rclpy = MagicMock(name="rclpy")
_mock_rclpy.spin = MagicMock()
_mock_rclpy.init = MagicMock()
_mock_rclpy.shutdown = MagicMock()
_mock_rclpy.ok = MagicMock(return_value=True)

_mock_geometry_msgs = MagicMock(name="geometry_msgs")
_mock_sensor_msgs = MagicMock(name="sensor_msgs")

sys.modules["rclpy"] = _mock_rclpy
sys.modules["geometry_msgs"] = _mock_geometry_msgs
sys.modules["geometry_msgs.msg"] = MagicMock()
sys.modules["sensor_msgs"] = _mock_sensor_msgs
sys.modules["sensor_msgs.msg"] = MagicMock()

# ---------------------------------------------------------------------------
# Mock Node 基类
# ---------------------------------------------------------------------------


class MockNode:
    """替代 rclpy.node.Node 的 mock 基类。"""

    _param_overrides: dict = {}

    def __init__(self, node_name: str = "", **kwargs):
        self._node_name = node_name
        self._declared_params = {}

    def declare_parameter(self, name, value=None):
        self._declared_params[name] = value

    def get_parameter(self, name):
        mock = MagicMock()
        if name in MockNode._param_overrides:
            mock.value = MockNode._param_overrides[name]
        elif name in self._declared_params:
            mock.value = self._declared_params[name]
        else:
            mock.value = name  # fallback
        return mock

    def get_logger(self):
        return MagicMock()

    def get_clock(self):
        mock = MagicMock()
        mock.now.return_value.nanoseconds = 0
        return mock

    def create_publisher(self, msg_type, topic, qos_profile):
        return MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        return MagicMock()

    def create_timer(self, timer_period_sec, callback):
        self._timer_callback = callback
        return MagicMock()

    def destroy_node(self):
        pass

    @classmethod
    def set_params(cls, **overrides):
        cls._param_overrides = dict(overrides)


_mock_rclpy.node.Node = MockNode

# 设置 mock 子模块
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.node"].Node = MockNode

# ---------------------------------------------------------------------------
# 导入被测试模块
# ---------------------------------------------------------------------------

# robot_control 包已在 sys.path（通过 conftest.py）
import robot_control.control_policy as _cp

sys.modules["robot_control.control_policy"] = _cp

from robot_control.cmd_vel_mux import CmdVelMuxNode


def make_mux(**param_overrides):
    """创建 CmdVelMuxNode 实例的工厂函数。"""
    MockNode.set_params(**param_overrides)
    with patch("os.path.exists", return_value=False):
        node = CmdVelMuxNode()
    node.publisher = MagicMock()
    node._logger = MagicMock()
    node.get_logger = MagicMock(return_value=node._logger)
    return node


# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------


class TestCmdVelMuxNode:
    """验证 CmdVelMuxNode 的参数和结构。"""

    def test_default_parameters_are_set(self):
        node = make_mux()
        assert node.get_parameter("linear_limit").value == 0.4
        assert node.get_parameter("angular_limit").value == 1.5
        assert node.get_parameter("timeout_sec").value == 0.25
        assert node.get_parameter("publish_hz").value == 20.0

    def test_research_sources_default_to_empty(self):
        node = make_mux()
        assert node.source_config.research_sources == ()

    def test_research_sources_are_passed_to_config(self):
        node = make_mux(research_sources=["avoidance", "mpc"])
        assert node.source_config.research_sources == ("avoidance", "mpc")

    def test_subscriptions_created_for_all_sources(self):
        node = make_mux(research_sources=["avoidance"])
        sources = set(node.subscriptions_by_source.keys())
        assert "teleop" in sources
        assert "nav" in sources
        assert "test" in sources
        assert "research/avoidance" in sources

    def test_driver_topic_is_configurable(self):
        node = make_mux(driver_topic="/cmd_vel/custom")
        assert node.get_parameter("driver_topic").value == "/cmd_vel/custom"


class TestCmdVelMuxPublish:
    """验证 _publish_selected 发布逻辑。"""

    def test_publishes_selected_command(self):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("teleop", Command(linear_x=0.3, angular_z=0.1), now_sec=1.0)

        node._publish_selected()

        node.publisher.publish.assert_called_once()
        msg = node.publisher.publish.call_args[0][0]
        assert msg.linear.x == 0.3
        assert msg.angular.z == 0.1

    def test_publishes_stop_when_no_active_source(self):
        node = make_mux()
        # 无活跃源 → idle → (0, 0)

        node._publish_selected()

        node.publisher.publish.assert_called_once()
        msg = node.publisher.publish.call_args[0][0]
        assert msg.linear.x == 0.0
        assert msg.angular.z == 0.0


class TestCmdVelMuxCallback:
    """验证 _make_callback 产生的回调函数。"""

    def test_callback_updates_mux_with_clamped_command(self):
        from robot_control.control_policy import Command

        node = make_mux()
        callback = node._make_callback("teleop")

        mock_twist = MagicMock()
        mock_twist.linear.x = 0.5
        mock_twist.angular.z = -0.2

        callback(mock_twist)

        selected = node.mux.select(now_sec=0.1)  # 在 0.25s 超时窗口内
        assert selected.source == "teleop"
        assert selected.command == Command(linear_x=0.4, angular_z=-0.2)
