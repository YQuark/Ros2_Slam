from pathlib import Path
import hashlib
import shutil

import pytest
import yaml

from robot_config.compiler import (
    ConfigError,
    _canonical_hash,
    compile_effective_config,
    load_effective_config,
)


ROOT = Path(__file__).resolve().parents[1] / "config"


def test_schema_targets_humble_jsonschema_draft7():
    schema = yaml.safe_load(
        (ROOT / "schemas" / "effective-config-v1.schema.yaml").read_text(encoding="utf-8")
    )
    assert schema["$schema"] == "http://json-schema.org/draft-07/schema#"


def test_effective_config_is_stable_and_api5_wire3(tmp_path):
    first = load_effective_config(ROOT)
    second = load_effective_config(ROOT)
    assert first["config_sha256"] == second["config_sha256"]
    assert first["platform_api_version"] == 5
    assert first["protocol"]["version"] == 3
    paths = compile_effective_config(ROOT, tmp_path / "compiled")
    params = yaml.safe_load(paths["ros_params"].read_text(encoding="utf-8"))
    assert params["stm32_bridge"]["ros__parameters"]["config_sha256"] == first["config_sha256"]
    odom = params["wheel_odometry"]["ros__parameters"]
    assert odom["hard_max_wheel_peripheral_speed_mps"] == 0.70
    assert odom["degraded_single_wheel_multiplier"] == 4.0
    manifest = yaml.safe_load(paths["manifest"].read_text(encoding="utf-8"))
    assert manifest["schema_version"] == 2
    for name, expected in manifest["artifact_sha256"].items():
        assert (
            hashlib.sha256((paths["manifest"].parent / name).read_bytes()).hexdigest() == expected
        )
    assert manifest["provenance"]["motion.hard_max_linear_mps"] == "safety.yaml"
    assert manifest["safety_override_audit"] == {"present": False, "paths": []}


def test_config_hash_excludes_only_its_self_referential_identity_fields():
    effective = load_effective_config(ROOT)
    expected = effective["config_sha256"]
    unhashed = yaml.safe_load(yaml.safe_dump(effective))
    unhashed.pop("config_sha256")
    unhashed["calibration_bundle"]["identity"].pop("upper_config_sha256")
    assert _canonical_hash(unhashed) == expected
    assert effective["calibration_bundle"]["identity"]["upper_config_sha256"] == expected


def test_timing_invariant_fails_closed():
    try:
        load_effective_config(
            ROOT,
            overrides={"safety": {"timing": {"drive_keepalive_sec": 0.19}}},
        )
    except ConfigError as exc:
        assert "keepalive" in str(exc)
    else:
        raise AssertionError("invalid timing must fail")


def test_release_mode_rejects_unsafe_and_provisional_calibration():
    for overrides, expected in (
        ({"release_mode": True}, "unsafe_development_mode"),
        ({"release_mode": True, "unsafe_development_mode": False}, "final calibration"),
    ):
        try:
            load_effective_config(ROOT, overrides=overrides)
        except ConfigError as exc:
            assert expected in str(exc)
        else:
            raise AssertionError("unsafe/provisional release config must fail")


def test_single_wheel_limit_covers_combined_hard_v_and_w_envelope():
    try:
        load_effective_config(
            ROOT,
            overrides={"calibration": {"limits": {"hard_max_wheel_peripheral_speed_mps": 0.50}}},
        )
    except ConfigError as exc:
        assert "single-wheel speed limit" in str(exc)
    else:
        raise AssertionError("undersized single-wheel speed limit must fail")


def test_odometry_noise_rejects_nonfinite_and_non_psd_correlation():
    for value in (float("nan"), 1.0, -1.0):
        try:
            load_effective_config(
                ROOT,
                overrides={"calibration": {"odom_covariance": {"left_right_correlation": value}}},
            )
        except ConfigError as exc:
            assert "left_right_correlation" in str(exc)
        else:
            raise AssertionError("invalid correlation must fail")


def test_schema_rejects_unknown_fields_and_nonfinite_values():
    for overrides, expected in (
        ({"motion": {"typo_limit": 1.0}}, "Additional properties"),
        ({"motion": {"hard_max_linear_mps": float("inf")}}, "non-finite"),
    ):
        try:
            load_effective_config(ROOT, overrides=overrides)
        except ConfigError as exc:
            assert expected in str(exc)
        else:
            raise AssertionError("unknown or nonfinite config must fail")


def test_physical_and_supervisor_invariants_fail_closed():
    cases = (
        ({"motion": {"max_linear_accel_mps2": -0.1}}, "must be positive"),
        (
            {"calibration_bundle": {"wheel_layout": {"expected_enabled_mask": 0b0010}}},
            "one wheel on each side",
        ),
        (
            {"calibration_bundle": {"geometry": {"wheel_radius_m": 0.04}}},
            "bundle geometry",
        ),
        (
            {"motion_supervisor": {"warn_score": 0.9}},
            "score thresholds",
        ),
        ({"runtime": {"mode": "navigation"}}, "runtime.mode"),
    )
    for overrides, expected in cases:
        with pytest.raises(ConfigError, match=expected):
            load_effective_config(ROOT, overrides=overrides)


def test_duplicate_yaml_key_is_rejected(tmp_path):
    config_root = tmp_path / "config"
    shutil.copytree(ROOT, config_root)
    platform = config_root / "platform.yaml"
    platform.write_text(
        platform.read_text(encoding="utf-8") + "\nrelease_mode: false\n",
        encoding="utf-8",
    )

    try:
        load_effective_config(config_root)
    except ConfigError as exc:
        assert "duplicate key" in str(exc)
    else:
        raise AssertionError("duplicate YAML key must fail")


def test_compile_is_atomic_and_refuses_existing_target(tmp_path):
    output = tmp_path / "compiled"
    compile_effective_config(ROOT, output)
    try:
        compile_effective_config(ROOT, output)
    except ConfigError as exc:
        assert "already exists" in str(exc)
    else:
        raise AssertionError("existing output target must not be overwritten")


def test_runtime_safety_overrides_are_audited(tmp_path):
    paths = compile_effective_config(
        ROOT,
        tmp_path / "compiled",
        overrides={"motion": {"soft_max_linear_mps": 0.20}},
    )
    manifest = yaml.safe_load(paths["manifest"].read_text(encoding="utf-8"))
    assert manifest["safety_override_audit"] == {
        "present": True,
        "paths": ["motion.soft_max_linear_mps"],
    }
