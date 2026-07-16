from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_platform_api_version_and_command_contract_are_explicit():
    assert (ROOT / "PLATFORM_API_VERSION").read_text(encoding="utf-8").strip() == "2"
    contract = yaml.safe_load((ROOT / "compatibility" / "firmware.yaml").read_text(encoding="utf-8"))
    assert contract["platform_api_version"] == 2
    assert contract["topics"]["chassis_command"] == "/chassis/command"
    assert contract["topics"]["legacy_driver_command"] == "/cmd_vel/driver"


def test_robot_interfaces_defines_command_and_state_messages():
    package = ROOT / "src" / "robot_interfaces"
    assert (package / "package.xml").is_file()
    assert (package / "CMakeLists.txt").is_file()

    command = (package / "msg" / "ChassisCommand.msg").read_text(encoding="utf-8")
    for declaration in (
        "std_msgs/Header header",
        "geometry_msgs/Twist twist",
        "bool enable",
        "uint8 source",
        "uint32 sequence",
        "uint8 SOURCE_NONE=0",
        "uint8 SOURCE_TELEOP=1",
        "uint8 SOURCE_TEST=2",
        "uint8 SOURCE_NAV=3",
        "uint8 SOURCE_RESEARCH=4",
    ):
        assert declaration in command

    state = (package / "msg" / "ChassisState.msg").read_text(encoding="utf-8")
    assert "uint8 bridge_state" in state
    assert "uint32 command_sequence" in state
    assert "uint8 firmware_control_source" in state


def test_api_v2_is_default_and_legacy_twist_is_opt_in():
    mux_config = yaml.safe_load(
        (ROOT / "src" / "robot_control" / "config" / "cmd_vel_mux.yaml").read_text(encoding="utf-8")
    )["cmd_vel_mux"]["ros__parameters"]
    assert mux_config["chassis_command_topic"] == "/chassis/command"
    assert mux_config["publish_legacy_twist"] is False

    launch = (ROOT / "src" / "stm32_robot_bridge" / "launch" / "stm32_bridge.launch.py").read_text(
        encoding="utf-8"
    )
    assert "DeclareLaunchArgument('chassis_command_topic', default_value='/chassis/command')" in launch
    assert "DeclareLaunchArgument('enable_legacy_cmd_vel', default_value='false')" in launch

    fake_base = (
        ROOT / "src" / "robot_state_estimation" / "scripts" / "fake_base_odom.py"
    ).read_text(encoding="utf-8")
    assert "declare_parameter('chassis_command_topic', '/chassis/command')" in fake_base
    assert "create_subscription(ChassisCommand" in fake_base
