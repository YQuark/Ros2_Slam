import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]


def _module():
    path = ROOT / "tools/calibration/analyze_motion_risk.py"
    spec = importlib.util.spec_from_file_location("analyze_motion_risk", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_motion_risk_contract_and_roc_are_explicitly_not_probability():
    contract = yaml.safe_load(
        (ROOT / "verification/configs/benchmarks/motion-risk-dataset-v0.6.0-rc2.yaml").read_text()
    )
    report = _module().analyze(
        [
            {"scenario": "normal", "fault_label": "0", "motion_consistency_risk": "0.1"},
            {"scenario": "turn", "fault_label": "0", "motion_consistency_risk": "0.3"},
            {"scenario": "stall", "fault_label": "1", "motion_consistency_risk": "0.8"},
            {"scenario": "push", "fault_label": "1", "motion_consistency_risk": "1.0"},
        ]
    )

    assert len(contract["scenarios"]) == 11
    assert report["candidate_threshold"]["fpr"] == 0.0
    assert report["candidate_threshold"]["tpr"] == 1.0
    assert report["roc_auc"] == pytest.approx(1.0)
    assert set(report["scenario_distributions"]) == {"normal", "turn", "stall", "push"}
    assert "not a calibrated probability" in report["terminology"]


def test_motion_risk_analysis_requires_both_classes():
    with pytest.raises(ValueError, match="both normal and fault"):
        _module().analyze(
            [{"scenario": "normal", "fault_label": "0", "motion_consistency_risk": "0.1"}]
        )
