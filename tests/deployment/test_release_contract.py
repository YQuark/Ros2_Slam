from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
FIRMWARE_COMMIT = "a4ae10d592e840e979dd35c8a9b2ff2f5dd107e4"


def test_firmware_contract_locks_beta4_and_real_upper_timeout() -> None:
    contract = yaml.safe_load(
        (ROOT / "compatibility" / "firmware.yaml").read_text(encoding="utf-8")
    )

    assert contract["upper"]["baseline_commit"] == "2c9718e7a62cbb0d1905e759073871d25621dba9"
    assert contract["upper"]["release"] == "v0.3.0"
    assert contract["firmware"]["tag"] == "v1.0.0-beta4"
    assert contract["firmware"]["commit"] == FIRMWARE_COMMIT
    assert contract["timing"]["firmware_upper_timeout_ms"] == 200
    assert contract["timing"]["deprecated_firmware_chassis_timeout_ms"] == 500
    assert contract["timing"]["bridge_keepalive_ms"] == 50
    assert contract["timing"]["bridge_command_timeout_ms"] == 150
    assert contract["timing"]["bridge_status_timeout_ms"] == 250
    assert (
        contract["timing"]["bridge_keepalive_ms"] < contract["timing"]["bridge_command_timeout_ms"]
    )
    assert (
        contract["timing"]["bridge_command_timeout_ms"]
        < contract["timing"]["firmware_upper_timeout_ms"]
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
    assert (ROOT / "docs" / "releases" / "v0.3.0.md").is_file()
    assert (ROOT / "reports" / "hil" / "v0.3.0.md").is_file()
    assert (ROOT / "reports" / "vehicle" / "v0.3.0.md").is_file()


def test_ci_enforces_quality_and_module_coverage_thresholds() -> None:
    ci = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")

    assert "ruff check" in ci
    assert "black --check" in ci
    assert "yamllint" in ci
    assert "shellcheck" in ci
    assert "coverage json -o coverage.json" in ci
    assert "scripts/verify/check_coverage.py" in ci


def test_all_owned_ros_packages_are_versioned_v030() -> None:
    packages = sorted((ROOT / "src").glob("*/package.xml"))
    assert packages
    assert {ET.parse(package).getroot().findtext("version") for package in packages} == {"0.3.0"}


def test_hil_contract_lists_all_fault_injections_and_is_fail_closed() -> None:
    contract = yaml.safe_load(
        (ROOT / "config" / "hil" / "fault_injection.yaml").read_text(encoding="utf-8")
    )
    assert len(contract["scenarios"]) == 13
    assert len({scenario["id"] for scenario in contract["scenarios"]}) == 13
    assert "Overall status: NOT_RUN" in (ROOT / "reports" / "hil" / "v0.3.0.md").read_text(
        encoding="utf-8"
    )
    assert "Overall status: NOT_RUN" in (ROOT / "reports" / "vehicle" / "v0.3.0.md").read_text(
        encoding="utf-8"
    )


def test_bridge_executor_path_contains_no_blocking_sleep() -> None:
    bridge = (
        ROOT / "src" / "stm32_robot_bridge" / "stm32_robot_bridge" / "bridge_node.py"
    ).read_text(encoding="utf-8")
    assert "time.sleep(" not in bridge
