from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_ekf_fuses_only_wheel_vx_and_imu_gyro_z_initially():
    config = yaml.safe_load(
        (ROOT / "src" / "robot_state_estimation" / "config" / "ekf_base.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]

    assert config["frequency"] == 30.0
    assert config["sensor_timeout"] == 0.2
    assert config["two_d_mode"] is True
    assert config["publish_tf"] is True
    assert config["odom0"] == "wheel/odom"
    assert [index for index, enabled in enumerate(config["odom0_config"]) if enabled] == [6]
    assert config["imu0"] == "imu/data"
    assert [index for index, enabled in enumerate(config["imu0_config"]) if enabled] == [11]
    assert config["imu0_remove_gravitational_acceleration"] is False
    assert len(config["process_noise_covariance"]) == 225
    assert len(config["initial_estimate_covariance"]) == 225


def test_real_base_tf_has_one_odom_to_base_link_owner():
    system = (ROOT / "src" / "robot_bringup" / "launch" / "system.launch.py").read_text(
        encoding="utf-8"
    )
    bridge_launch = (
        ROOT / "src" / "stm32_robot_bridge" / "launch" / "stm32_bridge.launch.py"
    ).read_text(encoding="utf-8")
    ekf = (ROOT / "src" / "robot_state_estimation" / "config" / "ekf_base.yaml").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument("base_fusion_mode", default_value="ekf")' in system
    assert "bridge v3 never owns odom->base_link TF" in (
        ROOT / "src/stm32_robot_bridge/stm32_robot_bridge/bridge_node_v3.py"
    ).read_text(encoding="utf-8")
    assert '"publish_tf": False' in (
        ROOT / "src/stm32_robot_bridge/stm32_robot_bridge/bridge_node_v3.py"
    ).read_text(encoding="utf-8")
    assert "publish_tf: true" in ekf
