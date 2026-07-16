from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
CALIBRATION = ROOT / "src" / "robot_bringup" / "config" / "robot_calibration.yaml"


def test_robot_calibration_is_versioned_and_records_provenance():
    calibration = yaml.safe_load(CALIBRATION.read_text(encoding="utf-8"))
    assert calibration["schema_version"] == 1
    assert calibration["calibration_version"]
    assert calibration["platform_api_version"] == 2
    assert calibration["status"] == "provisional"
    assert calibration["provenance"]["upper_commit"]
    assert calibration["provenance"]["firmware_commit"]


def test_runtime_defaults_match_canonical_calibration():
    calibration = yaml.safe_load(CALIBRATION.read_text(encoding="utf-8"))
    mux = yaml.safe_load(
        (ROOT / "src" / "robot_control" / "config" / "cmd_vel_mux.yaml").read_text(
            encoding="utf-8"
        )
    )["cmd_vel_mux"]["ros__parameters"]
    common = yaml.safe_load(
        (ROOT / "src" / "robot_bringup" / "config" / "robot_common.yaml").read_text(
            encoding="utf-8"
        )
    )["robot"]["ros__parameters"]

    assert mux["linear_limit"] == calibration["motion"]["soft_max_linear_mps"]
    assert mux["angular_limit"] == calibration["motion"]["soft_max_angular_radps"]
    assert mux["max_linear_accel"] == calibration["motion"]["max_linear_accel_mps2"]
    assert mux["max_angular_accel"] == calibration["motion"]["max_angular_accel_radps2"]
    assert mux["max_linear_jerk"] == calibration["motion"]["max_linear_jerk_mps3"]
    assert mux["max_angular_jerk"] == calibration["motion"]["max_angular_jerk_radps3"]
    assert common["wheel_radius_m"] == calibration["drive"]["wheel_radius_m"]
    assert common["wheel_track_width_m"] == calibration["drive"]["wheel_track_width_m"]
