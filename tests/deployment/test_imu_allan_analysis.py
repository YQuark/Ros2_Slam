import importlib.util
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]


def _module():
    path = ROOT / "tools/calibration/analyze_imu_allan.py"
    spec = importlib.util.spec_from_file_location("analyze_imu_allan", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_allan_analysis_emits_provisional_finite_estimates():
    rows = [
        {
            "timestamp_sec": f"{index * 0.01:.2f}",
            "gyro_z_radps": str(0.001 * math.sin(index * 0.13) + index * 1.0e-7),
            "temperature_c": str(20.0 + index * 0.001),
        }
        for index in range(1024)
    ]
    report = _module().analyze(rows)

    assert report["status"] == "provisional"
    assert report["sample_rate_hz"] == pytest.approx(100.0)
    assert len(report["allan_curve"]) >= 8
    assert report["estimates"]["recommended_gyro_variance"] >= 0.0


def test_allan_analysis_rejects_short_dataset():
    with pytest.raises(ValueError, match="128"):
        _module().analyze([])


def test_allan_analysis_rejects_nonuniform_sampling():
    rows = [
        {
            "timestamp_sec": str(index * 0.01 + (0.002 if index == 64 else 0.0)),
            "gyro_z_radps": "0.0",
            "temperature_c": "20.0",
        }
        for index in range(128)
    ]
    with pytest.raises(ValueError, match="jitter"):
        _module().analyze(rows)
