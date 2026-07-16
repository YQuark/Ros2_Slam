from pathlib import Path

import yaml

from robot_config.compiler import ConfigError, compile_effective_config, load_effective_config


ROOT = Path(__file__).resolve().parents[1] / "config"


def test_effective_config_is_stable_and_v3(tmp_path):
    first = load_effective_config(ROOT)
    second = load_effective_config(ROOT)
    assert first["config_sha256"] == second["config_sha256"]
    assert first["platform_api_version"] == 3
    assert first["protocol"]["version"] == 3
    paths = compile_effective_config(ROOT, tmp_path)
    params = yaml.safe_load(paths["ros_params"].read_text(encoding="utf-8"))
    assert params["stm32_bridge"]["ros__parameters"]["config_sha256"] == first["config_sha256"]


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
