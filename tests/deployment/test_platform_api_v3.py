from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_platform_api_v5_and_wire_v3_contract_are_explicit():
    assert (ROOT / "PLATFORM_API_VERSION").read_text(encoding="utf-8").strip() == "5"
    contract = yaml.safe_load(
        (ROOT / "compatibility" / "firmware.yaml").read_text(encoding="utf-8")
    )
    platform = yaml.safe_load(
        (ROOT / "src/robot_config/config/platform.yaml").read_text(encoding="utf-8")
    )
    assert contract["platform_api_version"] == 5
    assert contract["upper"]["wire_protocols_supported"] == [3]
    assert contract["firmware"]["status"] == "implementation_ready_hil_pending"
    assert platform["platform"]["topics"]["host_motion_command"] == "chassis/host_motion_command"


def test_robot_interfaces_define_session_ack_and_firmware_identity():
    package = ROOT / "src" / "robot_interfaces"
    command = (package / "msg" / "HostMotionCommand.msg").read_text(encoding="utf-8")
    for declaration in (
        "bool enable",
        "uint8 host_subsource",
        "uint64 command_epoch",
        "uint32 sequence",
    ):
        assert declaration in command
    state = (package / "msg" / "ChassisLinkState.msg").read_text(encoding="utf-8")
    for declaration in (
        "uint64 wire_session_id",
        "uint32 firmware_applied_sequence",
        "string config_sha256",
    ):
        assert declaration in state
    firmware = (package / "msg" / "FirmwareInfo.msg").read_text(encoding="utf-8")
    assert "string firmware_commit" in firmware
    assert "uint32 capabilities" in firmware
    assert "bool simulated" in firmware

    wheel = (package / "msg" / "WheelObservation.msg").read_text(encoding="utf-8")
    for declaration in (
        "uint8 schema_version",
        "uint64 transport_session_id",
        "uint32 status_sequence",
        "uint32 mcu_sample_time_ms",
        "int32[4] encoder_count",
        "uint8 speed_valid_mask",
        "uint32 error_flags",
    ):
        assert declaration in wheel
    imu = (package / "msg" / "ImuObservation.msg").read_text(encoding="utf-8")
    for declaration in (
        "uint32 sample_sequence",
        "float32[3] angular_velocity_dps",
        "uint32 quality_flags",
    ):
        assert declaration in imu
    control = (package / "msg" / "HostControlState.msg").read_text(encoding="utf-8")
    assert "uint8 command_reject_reason" in control
    assert "bool rearm_required" in control
    safety = (package / "msg" / "MotionSupervisionState.msg").read_text(encoding="utf-8")
    assert "float32 command_scale" in safety
    assert "bool release_host_candidate" in safety


def test_v5_runtime_is_effective_config_driven_and_legacy_twist_is_removed():
    mux = yaml.safe_load(
        (ROOT / "src/robot_control/config/cmd_vel_mux.yaml").read_text(encoding="utf-8")
    )
    assert (
        mux["cmd_vel_mux"]["ros__parameters"]["host_motion_command_topic"]
        == "chassis/host_motion_command"
    )
    launch = (ROOT / "src/stm32_robot_bridge/launch/stm32_bridge.launch.py").read_text(
        encoding="utf-8"
    )
    cmake = (ROOT / "src/stm32_robot_bridge/CMakeLists.txt").read_text(encoding="utf-8")
    assert "ROBOT_EFFECTIVE_PARAMS" in launch
    assert "bridge_node_v3.py" in cmake
    assert "RENAME bridge_node" in cmake
    assert "stm32_robot_bridge/protocol_v2.py" not in cmake
    assert "stm32_robot_bridge/bridge_node.py" not in cmake
    assert "enable_legacy_cmd_vel" not in launch
