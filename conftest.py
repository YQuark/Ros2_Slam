"""pytest 全局配置 —— ROS2 fixture、路径注入、通用工具。"""

from pathlib import Path
import sys
import pytest


# ---------------------------------------------------------------------------
# 路径注入 —— 确保 src/*/ 包可以直接 import
# ---------------------------------------------------------------------------

def pytest_configure(config):
    root = Path(__file__).resolve().parent
    for pkg_dir in (root / "src").iterdir():
        if not pkg_dir.is_dir():
            continue
        pkg_init = pkg_dir / "__init__.py"
        # ament_python 包有 __init__.py；ament_cmake_python 包也可能有
        if pkg_init.exists():
            sys.path.insert(0, str(pkg_dir))
        # 部分包的 Python 源码在包同名子目录下
        nested = pkg_dir / pkg_dir.name
        if nested.is_dir():
            sys.path.insert(0, str(pkg_dir))


# ---------------------------------------------------------------------------
# ROS2 运行时 fixture
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def ros_init():
    """会话级 ROS2 初始化，所有测试共享一个 rclpy 上下文。"""
    import rclpy
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


@pytest.fixture
def ros_node(ros_init):
    """函数级临时 ROS2 Node，测试结束后自动销毁。"""
    import rclpy
    node = rclpy.create_node("test_node_" + str(id(ros_init)))
    yield node
    node.destroy_node()
