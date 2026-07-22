from pathlib import Path

from robot_config.compiler import load_effective_config, ros_parameters


ROOT = Path(__file__).resolve().parents[2]
CONFIG_ROOT = ROOT / "src/robot_config/config"


def test_robot_calibration_is_versioned_provisional_and_executable():
    effective = load_effective_config(CONFIG_ROOT)
    assert effective["schema_version"] == 1
    assert effective["platform_api_version"] == 3
    assert effective["calibration"]["version"]
    assert effective["calibration"]["status"] == "provisional"
    assert effective["calibration"]["drive"]["encoder_counts_per_revolution"] > 0
    assert len(effective["config_sha256"]) == 64


def test_effective_ros_parameters_are_derived_from_canonical_calibration():
    effective = load_effective_config(CONFIG_ROOT)
    params = ros_parameters(effective)
    mux = params["cmd_vel_mux"]["ros__parameters"]
    bridge = params["stm32_bridge"]["ros__parameters"]
    wheel = params["wheel_odometry"]["ros__parameters"]
    assert mux["linear_limit"] == effective["motion"]["soft_max_linear_mps"]
    assert mux["max_linear_jerk"] == effective["motion"]["max_linear_jerk_mps3"]
    assert "wheel_radius" not in bridge
    assert wheel["wheel_radius"] == effective["calibration"]["drive"]["wheel_radius_m"]
    assert (
        wheel["encoder_counts_per_revolution"]
        == effective["calibration"]["drive"]["encoder_counts_per_revolution"]
    )
    assert bridge["config_sha256"] == wheel["config_sha256"] == effective["config_sha256"]
