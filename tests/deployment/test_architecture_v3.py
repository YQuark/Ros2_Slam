from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_robot_control_is_only_chassis_command_publisher():
    publishers = []
    for path in (ROOT / "src").glob("*/**/*.py"):
        if "/test/" in path.as_posix() or path.name.startswith("test_"):
            continue
        if "create_publisher(ChassisCommand" in path.read_text(encoding="utf-8"):
            publishers.append(path.relative_to(ROOT).as_posix())
    assert publishers == ["src/robot_control/robot_control/cmd_vel_mux.py"]


def test_bridge_has_no_navigation_dependency_and_only_installs_v3_runtime():
    package = (ROOT / "src/stm32_robot_bridge/package.xml").read_text(encoding="utf-8")
    cmake = (ROOT / "src/stm32_robot_bridge/CMakeLists.txt").read_text(encoding="utf-8")
    assert "nav2" not in package and "slam_toolbox" not in package
    assert "protocol_v3.py" in cmake and "bridge_node_v3.py" in cmake
    assert "protocol_v2.py" not in cmake and "bridge_node.py" not in cmake


def test_platform_topics_are_relative_and_namespace_is_launchable():
    platform = yaml.safe_load(
        (ROOT / "src/robot_config/config/platform.yaml").read_text(encoding="utf-8")
    )
    assert all(not value.startswith("/") for value in platform["platform"]["topics"].values())
    system = (ROOT / "src/robot_bringup/launch/system.launch.py").read_text(encoding="utf-8")
    assert "PushRosNamespace" in system
    assert 'DeclareLaunchArgument("namespace", default_value="")' in system


def test_no_certified_nav2_fallback_or_duplicated_slam_profile():
    config = ROOT / "src/robot_bringup/config"
    assert not (config / "nav2_params_fallback.yaml").exists()
    assert (config / "nav2_minimal_example.yaml").is_file()
    assert not list(config.glob("slam_toolbox_mapping_*.yaml"))
    for profile in (config / "slam_profiles").glob("*.yaml"):
        data = yaml.safe_load(profile.read_text(encoding="utf-8"))
        assert set(data) == {"profile", "overlay"}
    nav_profile = yaml.safe_load(
        (config / "nav2_profiles/navigation.yaml").read_text(encoding="utf-8")
    )
    assert set(nav_profile) == {"profile", "overlay"}


def test_real_base_launch_requires_compiled_effective_config():
    system = (ROOT / "src/robot_bringup/launch/system.launch.py").read_text(encoding="utf-8")
    bridge = (ROOT / "src/stm32_robot_bridge/launch/stm32_bridge.launch.py").read_text(
        encoding="utf-8"
    )
    assert "real base requires compiled effective params" in system
    assert "effective params are required; start through bin/robot" in bridge
