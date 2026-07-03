"""wheel_odom_republisher 单元测试 —— 验证 odom 重发布和 TF 转换。"""

import sys
from unittest.mock import MagicMock, patch

import pytest

# ---------------------------------------------------------------------------
# Mock rclpy + 依赖
# ---------------------------------------------------------------------------

_mock_rclpy = MagicMock(name="rclpy")
_mock_rclpy.init = MagicMock()
_mock_rclpy.shutdown = MagicMock()
_mock_rclpy.ok = MagicMock(return_value=True)
_mock_rclpy.spin = MagicMock()

_mock_geometry_msgs = MagicMock(name="geometry_msgs")
_mock_nav_msgs = MagicMock(name="nav_msgs")
_mock_tf2_ros = MagicMock(name="tf2_ros")

sys.modules["rclpy"] = _mock_rclpy
sys.modules["geometry_msgs"] = _mock_geometry_msgs
sys.modules["geometry_msgs.msg"] = MagicMock()
sys.modules["nav_msgs"] = _mock_nav_msgs
sys.modules["nav_msgs.msg"] = MagicMock()
sys.modules["tf2_ros"] = _mock_tf2_ros


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
            mock.value = name
        return mock

    def get_logger(self):
        return MagicMock()

    def create_publisher(self, msg_type, topic, qos_profile):
        return MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        self._odom_callback = callback
        return MagicMock()

    def destroy_node(self):
        pass

    @classmethod
    def set_params(cls, **overrides):
        cls._param_overrides = dict(overrides)


_mock_rclpy.node.Node = MockNode
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.node"].Node = MockNode

# 添加 scripts 目录到 path（wheel_odom_republisher 是独立脚本，非 Python 包）
import os as _os

_scripts_dir = _os.path.join(_os.path.dirname(__file__), "..", "..", "src", "robot_state_estimation", "scripts")
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

from wheel_odom_republisher import WheelOdomRepublisher


def make_republisher(**param_overrides):
    """创建 WheelOdomRepublisher 实例的工厂函数。"""
    MockNode.set_params(**param_overrides)
    node = WheelOdomRepublisher()
    node.publisher = MagicMock()
    node._logger = MagicMock()
    node.get_logger = MagicMock(return_value=node._logger)
    return node


# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------


class TestWheelOdomRepublisher:
    """验证 WheelOdomRepublisher 节点。"""

    def test_default_parameters(self):
        node = make_republisher()
        assert node.get_parameter("input_topic").value == "/wheel/odom"
        assert node.get_parameter("output_topic").value == "/odom"
        assert node.get_parameter("frame_id").value == "odom"
        assert node.get_parameter("child_frame_id").value == "base_link"
        assert node.get_parameter("publish_tf").value is True

    def test_custom_topics_are_configurable(self):
        node = make_republisher(
            input_topic="/custom/wheel/odom",
            output_topic="/custom/odom",
        )
        assert node.get_parameter("input_topic").value == "/custom/wheel/odom"
        assert node.get_parameter("output_topic").value == "/custom/odom"

    def test_on_odom_republishes_with_new_frame_ids(self):
        node = make_republisher(frame_id="odom", child_frame_id="base_link")

        mock_odom = MagicMock()
        mock_odom.header = MagicMock()
        mock_odom.header.stamp = MagicMock()
        mock_odom.header.frame_id = "wheel_odom"
        mock_odom.child_frame_id = "base_link_wheel"
        mock_odom.pose = MagicMock()
        mock_odom.twist = MagicMock()

        node._on_odom(mock_odom)

        node.publisher.publish.assert_called_once()
        out = node.publisher.publish.call_args[0][0]
        assert out.header.frame_id == "odom"
        assert out.child_frame_id == "base_link"
        assert out.pose is mock_odom.pose
        assert out.twist is mock_odom.twist

    def test_tf_is_published_when_publish_tf_is_true(self):
        node = make_republisher(publish_tf=True)
        node.tf_broadcaster = MagicMock()

        mock_odom = MagicMock()
        mock_odom.header = MagicMock()
        mock_odom.header.stamp = MagicMock()
        mock_odom.child_frame_id = "base_link"
        mock_odom.pose.pose.position.x = 1.0
        mock_odom.pose.pose.position.y = 0.5
        mock_odom.pose.pose.position.z = 0.0
        mock_odom.pose.pose.orientation = MagicMock()

        node._on_odom(mock_odom)

        node.tf_broadcaster.sendTransform.assert_called_once()

    def test_tf_is_not_published_when_publish_tf_is_false(self):
        node = make_republisher(publish_tf=False)
        # tf_broadcaster 应为 None
        assert node.tf_broadcaster is None

        mock_odom = MagicMock()
        mock_odom.header = MagicMock()
        mock_odom.pose.pose.position.x = 1.0
        mock_odom.pose.pose.orientation = MagicMock()

        node._on_odom(mock_odom)

        # 不应崩溃
        node.publisher.publish.assert_called_once()
