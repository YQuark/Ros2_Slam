from pathlib import Path
import xml.etree.ElementTree as ET
import importlib.util

import yaml


ROOT = Path(__file__).resolve().parents[2]
FIRMWARE_COMMIT = "a4ae10d592e840e979dd35c8a9b2ff2f5dd107e4"


def test_firmware_contract_requires_v3_and_marks_beta4_incompatible() -> None:
    contract = yaml.safe_load(
        (ROOT / "compatibility" / "firmware.yaml").read_text(encoding="utf-8")
    )

    assert contract["upper"]["baseline_commit"] == "b70721c0e8e8bf3aa5c76debc0b806474a546a26"
    assert contract["upper"]["release"] == "v0.4.0"
    assert contract["upper"]["wire_protocols_supported"] == [3]
    assert contract["firmware"]["compatible_commit"] is None
    assert contract["incompatible_firmware"][0]["commit"] == FIRMWARE_COMMIT
    assert contract["timing"]["required_firmware_upper_timeout_ms"] == 200
    assert contract["timing"]["bridge_keepalive_ms"] == 50
    assert contract["timing"]["bridge_command_timeout_ms"] == 150
    assert contract["timing"]["bridge_status_timeout_ms"] == 250
    assert (
        contract["timing"]["bridge_keepalive_ms"] < contract["timing"]["bridge_command_timeout_ms"]
    )
    assert (
        contract["timing"]["bridge_command_timeout_ms"]
        < contract["timing"]["required_firmware_upper_timeout_ms"]
    )


def test_unified_verify_entrypoints_exist_and_use_python_module_pytest() -> None:
    for name in ("verify_upper.sh", "verify_protocol.sh", "verify_runtime.sh"):
        path = ROOT / "scripts" / "verify" / name
        assert path.is_file(), name
        assert path.stat().st_mode & 0o111, name

    upper_script = (ROOT / "scripts" / "verify" / "verify_upper.sh").read_text(encoding="utf-8")
    assert "python3 -m pytest" in upper_script


def test_release_documentation_entrypoints_exist() -> None:
    assert (ROOT / "CHANGELOG.md").is_file()
    assert (ROOT / "docs" / "releases" / "v0.4.0.md").is_file()
    assert (ROOT / "verification" / "reports" / "hil" / "v0.4.0.yaml").is_file()
    assert (ROOT / "verification" / "reports" / "vehicle" / "v0.4.0.yaml").is_file()


def test_ci_enforces_quality_and_module_coverage_thresholds() -> None:
    ci = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")

    assert "ruff check" in ci
    assert "black --check" in ci
    assert "yamllint" in ci
    assert "shellcheck" in ci
    assert "coverage json -o coverage.json" in ci
    assert "scripts/verify/check_coverage.py" in ci


def test_all_owned_ros_packages_are_versioned_v040() -> None:
    packages = sorted((ROOT / "src").glob("*/package.xml"))
    assert packages
    assert {ET.parse(package).getroot().findtext("version") for package in packages} == {"0.4.0"}


def test_hil_contract_lists_all_fault_injections_and_is_fail_closed() -> None:
    contract = yaml.safe_load(
        (ROOT / "verification/configs/hil/fault-injection-v0.4.0.yaml").read_text(encoding="utf-8")
    )
    assert len(contract["scenarios"]) == 16
    assert len({scenario["id"] for scenario in contract["scenarios"]}) == 16
    hil = yaml.safe_load(
        (ROOT / "verification/reports/hil/v0.4.0.yaml").read_text(encoding="utf-8")
    )
    vehicle = yaml.safe_load(
        (ROOT / "verification/reports/vehicle/v0.4.0.yaml").read_text(encoding="utf-8")
    )
    assert hil["result"] == vehicle["result"] == "NOT_RUN"


def test_bridge_executor_path_contains_no_blocking_sleep() -> None:
    bridge = (
        ROOT / "src" / "stm32_robot_bridge" / "stm32_robot_bridge" / "bridge_node_v3.py"
    ).read_text(encoding="utf-8")
    assert "time.sleep(" not in bridge


def test_v040_release_gate_is_machine_readable_and_fail_closed() -> None:
    path = ROOT / "scripts/verify/verify_release.py"
    spec = importlib.util.spec_from_file_location("verify_release", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    failures = module.release_failures(ROOT)
    assert any("hil_report is not PASS: NOT_RUN" in failure for failure in failures)
    assert any(
        "firmware compatibility is not release-compatible" in failure for failure in failures
    )
