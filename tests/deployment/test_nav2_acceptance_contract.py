from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "verification/configs/experiments/nav2-acceptance-v0.6.0-rc2.yaml"


def test_nav2_acceptance_requires_dynamics_and_fifty_repeatable_trials():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert config["vehicle_characterization"]["required_status"] == "PASS"
    assert len(config["vehicle_characterization"]["required_measurements"]) == 8
    assert len(config["targets"]) == 10
    assert config["repetitions_per_target"] == 5
    assert config["acceptance"]["total_trials"]["limit"] == 50
    assert config["acceptance"]["collision_count"]["limit"] == 0
    assert config["acceptance"]["communication_degradation_count"]["limit"] == 0
    assert config["fixed_map_sha256"] is None
    assert config["status"] == "dynamics_and_dataset_pending"
