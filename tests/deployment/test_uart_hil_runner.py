import importlib.util
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def _module():
    path = ROOT / "verification/runners/hil/run_uart_hil.py"
    spec = importlib.util.spec_from_file_location("run_uart_hil", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_uart_hil_contract_has_twenty_unique_scenarios_and_timing_evidence():
    contract = yaml.safe_load((ROOT / "verification/configs/hil/uart-v0.6.0-rc2.yaml").read_text())
    scenarios = contract["scenarios"]

    assert len(scenarios) == 20
    assert len({scenario["id"] for scenario in scenarios}) == 20
    timed = [scenario for scenario in scenarios if "max_motor_target_zero_ms" in scenario]
    assert len(timed) >= 10
    assert all("wheel_speed_below_threshold_at" in item["required_events"] for item in timed)


def test_hil_evaluator_fails_closed_on_missing_identity_and_events():
    contract = yaml.safe_load((ROOT / "verification/configs/hil/uart-v0.6.0-rc2.yaml").read_text())
    failures, metrics = _module().evaluate(contract, {}, {})

    assert metrics["scenario_count"] == 20
    assert any("upper_commit" in failure for failure in failures)
    assert any("power_on_release missing events" in failure for failure in failures)


def test_hil_evaluator_rejects_zero_crc_invalid_artifacts_and_equal_timestamps():
    contract = {
        "firmware_candidate_commit": "b" * 40,
        "scenarios": [
            {"id": "stop", "required_events": ["fault_injected_at", "motor_target_zero_at"]}
        ],
    }
    metadata = {
        "upper_commit": "a" * 40,
        "firmware_commit": "b" * 40,
        "hardware_revision": 1,
        "parameter_crc32": 0,
        "config_sha256": "c" * 64,
        "artifact_sha256": {"ros-params.yaml": "invalid"},
        "calibration_version": "chassis-v1",
    }
    events = {"stop": {"fault_injected_at": 10, "motor_target_zero_at": 10}}

    failures, _ = _module().evaluate(contract, metadata, events)

    assert any("non-zero" in failure for failure in failures)
    assert any("invalid artifact" in failure for failure in failures)
    assert any("strictly increasing" in failure for failure in failures)
