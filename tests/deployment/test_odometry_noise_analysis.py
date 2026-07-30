import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]


def _module():
    path = ROOT / "tools/calibration/analyze_odometry_noise.py"
    spec = importlib.util.spec_from_file_location("analyze_odometry_noise", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_odometry_noise_analysis_is_provisional_and_condition_aware():
    rows = [
        {
            "condition": "tile_low",
            "distance_m": "1",
            "left_error_m": "0.01",
            "right_error_m": "0.02",
        },
        {
            "condition": "tile_low",
            "distance_m": "2",
            "left_error_m": "0.02",
            "right_error_m": "0.03",
        },
        {
            "condition": "carpet_turn",
            "distance_m": "5",
            "left_error_m": "0.08",
            "right_error_m": "0.10",
        },
    ]
    report = _module().analyze(rows)

    assert report["status"] == "provisional"
    assert report["trial_count"] == 3
    assert set(report["metrics"]["condition_p95_abs_error_m"]) == {
        "carpet_turn",
        "tile_low",
    }
    assert report["candidate_odometry_noise"]["left_variance_per_meter"] >= 0.0
    assert "left_scale_error_per_meter" in report["metrics"]


def test_odometry_noise_detrends_deterministic_scale_error():
    rows = [
        {
            "condition": "tile",
            "distance_m": str(distance),
            "left_error_m": str(0.01 + 0.02 * distance),
            "right_error_m": str(-0.01 + 0.03 * distance),
        }
        for distance in (1, 2, 3, 4, 5)
    ]
    report = _module().analyze(rows)
    candidate = report["candidate_odometry_noise"]
    assert candidate["left_variance_floor_m2"] == pytest.approx(0.0, abs=1e-15)
    assert candidate["right_variance_floor_m2"] == pytest.approx(0.0, abs=1e-15)


def test_odometry_noise_analysis_rejects_too_little_or_nonfinite_data():
    with pytest.raises(ValueError, match="three trials"):
        _module().analyze([])
    with pytest.raises(ValueError, match="finite"):
        _module().analyze(
            [
                {"condition": "x", "distance_m": "1", "left_error_m": "nan", "right_error_m": "0"},
                {"condition": "x", "distance_m": "2", "left_error_m": "0", "right_error_m": "0"},
                {"condition": "x", "distance_m": "3", "left_error_m": "0", "right_error_m": "0"},
            ]
        )
