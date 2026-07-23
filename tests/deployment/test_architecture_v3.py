from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_robot_control_is_only_host_motion_command_publisher():
    publishers = []
    for path in (ROOT / "src").glob("*/**/*.py"):
        if "/test/" in path.as_posix() or path.name.startswith("test_"):
            continue
        if "create_publisher(HostMotionCommand" in path.read_text(encoding="utf-8"):
            publishers.append(path.relative_to(ROOT).as_posix())
    assert publishers == ["src/robot_control/robot_control/cmd_vel_mux.py"]


def test_bridge_has_no_navigation_dependency_and_only_installs_v3_runtime():
    package = (ROOT / "src/stm32_robot_bridge/package.xml").read_text(encoding="utf-8")
    cmake = (ROOT / "src/stm32_robot_bridge/CMakeLists.txt").read_text(encoding="utf-8")
    assert "nav2" not in package and "slam_toolbox" not in package
    assert "protocol_v3.py" in cmake and "bridge_node_v3.py" in cmake
    assert "protocol_v2.py" not in cmake and "bridge_node.py" not in cmake


def test_bridge_runtime_has_no_odometry_supervision_or_tf_ownership():
    node = (ROOT / "src/stm32_robot_bridge/stm32_robot_bridge/bridge_node_v3.py").read_text(
        encoding="utf-8"
    )
    for forbidden in (
        "EncoderOdometry",
        "MotionSupervisor",
        "nav_msgs",
        "TransformBroadcaster",
        "create_publisher(Odometry",
    ):
        assert forbidden not in node
    assert "WheelObservation" in node and "ImuObservation" in node


def test_fake_and_real_base_share_raw_platform_contract_and_publish_no_tf():
    real = (ROOT / "src/stm32_robot_bridge/stm32_robot_bridge/bridge_node_v3.py").read_text(
        encoding="utf-8"
    )
    fake = (ROOT / "src/robot_verification/robot_verification/fake_base_node.py").read_text(
        encoding="utf-8"
    )
    for message in (
        "HostMotionCommand",
        "WheelObservation",
        "ImuObservation",
        "ChassisLinkState",
        "FirmwareControlState",
        "FirmwareInfo",
        "DiagnosticArray",
    ):
        assert message in real and message in fake
    for provider in (real, fake):
        assert "Odometry" not in provider
        assert "TransformBroadcaster" not in provider
    assert '"chassis/get_info"' in fake
    assert '"chassis/line_ctrl"' in fake


def test_state_estimation_is_only_wheel_odom_owner_and_ekf_is_only_dynamic_tf_owner():
    wheel = (
        ROOT / "src/robot_state_estimation/robot_state_estimation/wheel_odometry_node.py"
    ).read_text(encoding="utf-8")
    assert "create_publisher(" in wheel and "Odometry" in wheel
    assert "TransformBroadcaster" not in wheel
    for config_name in ("ekf_wheel.yaml", "ekf_base.yaml"):
        config = yaml.safe_load(
            (ROOT / "src/robot_state_estimation/config" / config_name).read_text(encoding="utf-8")
        )
        assert config["/**"]["ros__parameters"]["publish_tf"] is True


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
    assert "base providers require compiled effective params" in system
    assert "effective params are required; start through bin/robot" in bridge
