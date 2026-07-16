import importlib.util
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "config" / "experiments" / "nav2_acceptance.yaml"
MODULE = ROOT / "tools" / "experiments" / "navigation_gate.py"


def load_module():
    spec = importlib.util.spec_from_file_location("navigation_gate", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_runs():
    return [
        {
            "target_id": f"target_{target:02d}",
            "success": target != 10,
            "position_error_m": 0.10,
            "yaw_error_rad": 0.20,
            "recovery_count": 0,
            "collision": False,
            "manual_intervention": False,
            "unexpected_motion": False,
        }
        for target in range(1, 11)
        for repetition in range(5)
    ]


def test_navigation_contract_requires_vehicle_characterization_and_nine_axes():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

    assert config["schema_version"] == 1
    assert config["vehicle_characterization"]["status"] == "measurement_required"
    assert set(config["vehicle_characterization"]["required_measurements"]) == {
        "minimum_stable_linear_mps",
        "minimum_stable_angular_radps",
        "startup_deadband_mps",
        "braking_distance_m",
        "maximum_stable_linear_accel_mps2",
        "maximum_stable_angular_accel_radps2",
    }
    assert set(config["parameter_axes"]) == {
        "min_speed_xy",
        "min_speed_theta",
        "acc_lim_x",
        "acc_lim_theta",
        "decel_lim_x",
        "decel_lim_theta",
        "xy_goal_tolerance",
        "yaw_goal_tolerance",
        "trans_stopped_velocity",
    }
    assert len(config["targets"]) == 10
    assert config["repetitions_per_target"] == 5

    baseline = yaml.safe_load((ROOT / config["baseline_params"]).read_text(encoding="utf-8"))
    for parameter, values in config["parameter_axes"].items():
        for path in config["parameter_paths"][parameter]:
            current = baseline
            for key in path:
                current = current[key]
            assert current == values[0], f"baseline drift: {parameter} at {path}"


def test_45_of_50_clean_runs_pass_strict_navigation_gate():
    module = load_module()
    config = module.load_contract(CONFIG)

    result = module.evaluate_runs(config, make_runs())

    assert result.passed is True
    assert result.metrics["total_trials"] == 50
    assert result.metrics["successful_trials"] == 45
    assert result.metrics["success_rate_percent"] == 90.0


def test_collision_intervention_or_incomplete_target_repetition_fails():
    module = load_module()
    config = module.load_contract(CONFIG)
    collision_runs = make_runs()
    collision_runs[0]["collision"] = True
    assert module.evaluate_runs(config, collision_runs).passed is False
    assert "collision_count" in module.evaluate_runs(config, collision_runs).failed

    intervention_runs = make_runs()
    intervention_runs[0]["manual_intervention"] = True
    assert "manual_intervention_count" in module.evaluate_runs(config, intervention_runs).failed

    incomplete = module.evaluate_runs(config, make_runs()[:-1])
    assert incomplete.passed is False
    assert incomplete.coverage_errors

    unset = module.evaluate_runs(
        config,
        [{"target_id": "target_01", "success": None, "recovery_count": None}],
    )
    assert unset.passed is False
    assert unset.coverage_errors
