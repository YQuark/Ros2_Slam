import importlib.util
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "verification" / "configs" / "experiments" / "slam-sweep-v0.6.0-rc2.yaml"
MODULE = ROOT / "tools" / "experiments" / "experiment_gate.py"


def load_module():
    spec = importlib.util.spec_from_file_location("experiment_gate", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_slam_sweep_covers_requested_parameters_datasets_and_metrics():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

    assert config["schema_version"] == 1
    assert config["strategy"] == "latin_hypercube_then_interaction"
    assert config["seed"] == 42
    assert config["coarse_candidate_count"] == 32
    assert set(config["training_datasets"]) == {"bag_06_mapping_train"}
    assert set(config["validation_datasets"]) == {"bag_07_mapping_validation"}
    assert config["evaluation_policy"] == {
        "tune_on_validation": False,
        "screenshot_is_evidence": False,
        "require_raw_metrics": True,
    }
    assert set(config["parameter_axes"]) == {
        "minimum_travel_distance",
        "minimum_travel_heading",
        "minimum_time_interval",
        "throttle_scans",
        "resolution",
        "loop_search_maximum_distance",
    }
    assert set(config["acceptance"]) == {
        "closure_position_error_m",
        "closure_yaw_error_rad",
        "wall_ghost_width_p95_m",
        "false_loop_closures",
        "map_node_count",
        "cpu_p95_percent",
        "memory_peak_mb",
        "tf_latency_p95_sec",
        "scan_drop_rate_percent",
    }


def test_latin_hypercube_is_deterministic_and_covers_each_axis():
    spec = importlib.util.spec_from_file_location(
        "latin_hypercube", ROOT / "tools/experiments/latin_hypercube.py"
    )
    lhs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lhs)
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    first = lhs.generate_lhs(
        config["parameter_axes"], config["coarse_candidate_count"], config["seed"]
    )
    second = lhs.generate_lhs(
        config["parameter_axes"], config["coarse_candidate_count"], config["seed"]
    )
    assert first == second
    assert len(first) == 32
    assert all(set(candidate["parameters"]) == set(config["parameter_axes"]) for candidate in first)


def test_strict_gate_rejects_missing_or_out_of_limit_metric():
    module = load_module()
    config = module.load_experiment(CONFIG)
    passing = {metric: rule["limit"] for metric, rule in config["acceptance"].items()}

    assert module.evaluate_metrics(config, passing).passed is True

    missing = dict(passing)
    del missing["tf_latency_p95_sec"]
    missing_result = module.evaluate_metrics(config, missing)
    assert missing_result.passed is False
    assert missing_result.missing == ("tf_latency_p95_sec",)

    unset = dict(passing)
    unset["memory_peak_mb"] = None
    assert module.evaluate_metrics(config, unset).missing == ("memory_peak_mb",)

    non_finite = dict(passing)
    non_finite["cpu_p95_percent"] = float("nan")
    assert "cpu_p95_percent" in module.evaluate_metrics(config, non_finite).failed

    failing = dict(passing)
    failing["false_loop_closures"] = 1
    failed_result = module.evaluate_metrics(config, failing)
    assert failed_result.passed is False
    assert "false_loop_closures" in failed_result.failed
