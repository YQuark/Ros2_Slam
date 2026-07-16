import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT / "scripts" / "verify" / "check_coverage.py"


def load_module():
    spec = importlib.util.spec_from_file_location("check_coverage", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def coverage(percentages, total=80.0):
    return {
        "totals": {"percent_covered": total},
        "files": {
            f"prefix/{name}": {"summary": {"percent_covered": value}}
            for name, value in percentages.items()
        },
    }


def test_module_coverage_gate_enforces_core_and_total_thresholds():
    module = load_module()
    passing = coverage(
        {
            "control_policy.py": 90.0,
            "protocol_v2.py": 91.0,
            "odometry.py": 99.0,
            "bridge_node.py": 85.0,
        },
        total=75.0,
    )
    assert module.coverage_failures(passing) == []

    failing = coverage(
        {
            "control_policy.py": 89.9,
            "protocol_v2.py": 91.0,
            "odometry.py": 99.0,
            "bridge_node.py": 84.9,
        },
        total=74.9,
    )
    failures = module.coverage_failures(failing)
    assert any("control_policy.py" in failure for failure in failures)
    assert any("bridge_node.py" in failure for failure in failures)
    assert any("TOTAL" in failure for failure in failures)
