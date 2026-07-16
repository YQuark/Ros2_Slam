"""Compile canonical YAML fragments into one validated effective configuration."""

from __future__ import annotations

import hashlib
import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

import yaml


class ConfigError(ValueError):
    pass


def _load(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"cannot load config {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ConfigError(f"config root must be a mapping: {path}")
    return value


def _merge(base: dict[str, Any], overlay: Mapping[str, Any]) -> dict[str, Any]:
    result = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, Mapping) and isinstance(result.get(key), Mapping):
            result[key] = _merge(dict(result[key]), value)
        else:
            result[key] = deepcopy(value)
    return result


def _canonical_hash(config: Mapping[str, Any]) -> str:
    payload = json.dumps(config, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _positive(config: Mapping[str, Any], path: str) -> float:
    value: Any = config
    for part in path.split("."):
        if not isinstance(value, Mapping) or part not in value:
            raise ConfigError(f"missing required config: {path}")
        value = value[part]
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{path} must be numeric") from exc
    if converted <= 0.0:
        raise ConfigError(f"{path} must be positive")
    return converted


def validate_effective_config(config: Mapping[str, Any]) -> None:
    if int(config.get("schema_version", 0)) != 1:
        raise ConfigError("schema_version must be 1")
    if int(config.get("platform_api_version", 0)) != 3:
        raise ConfigError("platform_api_version must be 3")
    if int(config.get("protocol", {}).get("version", 0)) != 3:
        raise ConfigError("only upper protocol v3 is supported")

    keepalive = _positive(config, "safety.timing.drive_keepalive_sec")
    bridge_timeout = _positive(config, "safety.timing.bridge_command_timeout_sec")
    firmware_timeout = _positive(config, "safety.timing.firmware_upper_timeout_sec")
    if not keepalive < bridge_timeout < firmware_timeout:
        raise ConfigError("require keepalive < bridge timeout < firmware timeout")
    if _positive(config, "safety.timing.command_ack_timeout_sec") > bridge_timeout:
        raise ConfigError("command ACK timeout must not exceed bridge command timeout")

    motion = config.get("motion", {})
    if float(motion.get("soft_max_linear_mps", 0.0)) > float(
        motion.get("hard_max_linear_mps", 0.0)
    ):
        raise ConfigError("soft linear limit exceeds hard limit")
    if float(motion.get("soft_max_angular_radps", 0.0)) > float(
        motion.get("hard_max_angular_radps", 0.0)
    ):
        raise ConfigError("soft angular limit exceeds hard limit")

    footprint = config.get("hardware", {}).get("footprint_m", [])
    if not isinstance(footprint, list) or len(footprint) < 3:
        raise ConfigError("hardware.footprint_m must contain at least three points")
    if config.get("release_mode") and config.get("unsafe_development_mode"):
        raise ConfigError("release mode forbids unsafe_development_mode")
    calibration = config.get("calibration", {})
    if config.get("release_mode") and calibration.get("status") != "final":
        raise ConfigError("release mode requires final calibration")


def load_effective_config(
    config_root: Path,
    *,
    robot_id: str = "robot_001",
    profile: str = "mapping",
    overrides: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    files = (
        config_root / "platform.yaml",
        config_root / "safety.yaml",
        config_root / "hardware" / f"{robot_id}.yaml",
        config_root / "calibration" / robot_id / "chassis_v1.yaml",
        config_root / "calibration" / robot_id / "imu_v1.yaml",
        config_root / "profiles" / f"{profile}.yaml",
    )
    effective: dict[str, Any] = {}
    for path in files:
        effective = _merge(effective, _load(path))
    if overrides:
        effective = _merge(effective, overrides)
    effective["robot_id"] = robot_id
    effective["profile"] = profile
    validate_effective_config(effective)
    without_hash = dict(effective)
    without_hash.pop("config_sha256", None)
    effective["config_sha256"] = _canonical_hash(without_hash)
    return effective


def ros_parameters(config: Mapping[str, Any]) -> dict[str, Any]:
    timing = config["safety"]["timing"]
    motion = config["motion"]
    drive = config["calibration"]["drive"]
    covariance = config["calibration"]["odom_covariance"]
    imu = config["calibration"]["imu"]
    supervisor = config["motion_supervisor"]
    topics = config["platform"]["topics"]
    frames = config["platform"]["frames"]
    common = {
        "config_sha256": config["config_sha256"],
        "unsafe_development_mode": bool(config.get("unsafe_development_mode", False)),
    }
    return {
        "stm32_bridge": {
            "ros__parameters": {
                **common,
                "port": config["hardware"]["base_port"],
                "baudrate": config["hardware"]["baudrate"],
                "protocol_version": 3,
                "chassis_command_topic": topics["chassis_command"],
                "odom_topic": topics["wheel_odom"],
                "imu_topic": topics["imu"],
                "frame_id": frames["odom"],
                "child_frame_id": frames["base"],
                "imu_frame_id": frames["imu"],
                "publish_tf": False,
                "cmd_timeout": timing["bridge_command_timeout_sec"],
                "drive_keepalive_sec": timing["drive_keepalive_sec"],
                "status_timeout": timing["status_timeout_sec"],
                "command_ack_timeout_sec": timing["command_ack_timeout_sec"],
                "max_command_age_sec": timing["max_command_age_sec"],
                "hard_max_linear_mps": motion["hard_max_linear_mps"],
                "hard_max_angular_radps": motion["hard_max_angular_radps"],
                "wheel_radius": drive["wheel_radius_m"],
                "wheel_track_width": drive["wheel_track_width_m"],
                "encoder_counts_per_revolution": drive["encoder_counts_per_revolution"],
                "odom_linear_scale": drive["odom_linear_scale"],
                "odom_angular_scale": drive["odom_angular_scale"],
                "odom_angular_sign": drive["odom_angular_sign"],
                **{f"odom_covariance.{key}": value for key, value in covariance.items()},
                **{f"motion_supervisor.{key}": value for key, value in supervisor.items()},
                "imu.use_orientation": imu["use_orientation"],
                "imu.orientation_stddev": imu["orientation_stddev"],
                "imu.angular_velocity_stddev": imu["angular_velocity_stddev"],
                "imu.linear_acceleration_stddev": imu["linear_acceleration_stddev"],
            }
        },
        "cmd_vel_mux": {
            "ros__parameters": {
                **common,
                "chassis_command_topic": topics["chassis_command"],
                "linear_limit": motion["soft_max_linear_mps"],
                "angular_limit": motion["soft_max_angular_radps"],
                "max_linear_accel": motion["max_linear_accel_mps2"],
                "max_angular_accel": motion["max_angular_accel_radps2"],
                "max_linear_jerk": motion["max_linear_jerk_mps3"],
                "max_angular_jerk": motion["max_angular_jerk_radps3"],
                "timeout_sec": timing["mux_source_timeout_sec"],
                "publish_hz": 20.0,
            }
        },
    }


def compile_effective_config(
    config_root: Path,
    output_dir: Path,
    *,
    robot_id: str = "robot_001",
    profile: str = "mapping",
    overrides: Mapping[str, Any] | None = None,
) -> dict[str, Path]:
    effective = load_effective_config(
        config_root, robot_id=robot_id, profile=profile, overrides=overrides
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    effective_path = output_dir / "effective-config.yaml"
    ros_path = output_dir / "ros-params.yaml"
    manifest_path = output_dir / "manifest.yaml"
    effective_path.write_text(yaml.safe_dump(effective, sort_keys=True), encoding="utf-8")
    ros_path.write_text(
        yaml.safe_dump(ros_parameters(effective), sort_keys=False), encoding="utf-8"
    )
    manifest = {
        "schema_version": 1,
        "robot_id": robot_id,
        "profile": profile,
        "config_sha256": effective["config_sha256"],
        "effective_config": str(effective_path),
        "ros_params": str(ros_path),
    }
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return {"effective": effective_path, "ros_params": ros_path, "manifest": manifest_path}
