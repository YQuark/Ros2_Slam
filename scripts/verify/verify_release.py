#!/usr/bin/env python3
"""Fail-closed v0.6.0-rc2 release gate over machine-readable evidence."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
RELEASE = "v0.6.0-rc2"
MANIFEST = ROOT / "verification" / "configs" / "release" / f"{RELEASE}.yaml"
SHA256 = re.compile(r"^[0-9a-f]{64}$")
GIT_SHA = re.compile(r"^[0-9a-f]{40}$")


def _mapping(path: Path) -> dict:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot load {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"document is not a mapping: {path}")
    return value


def _validate_result(name: str, report: dict, release: str) -> list[str]:
    failures = []
    required = {
        "schema_version",
        "release",
        "result",
        "upper_commit",
        "firmware_commit",
        "hardware_revision",
        "config_sha256",
        "artifact_sha256",
        "parameter_crc32",
        "calibration_version",
        "metrics",
    }
    missing = sorted(required - set(report))
    if missing:
        failures.append(f"{name} missing fields: {', '.join(missing)}")
        return failures
    if report["schema_version"] != 1 or report["release"] != release:
        failures.append(f"{name} schema/release mismatch")
    if report["result"] != "PASS":
        failures.append(f"{name} is not PASS: {report['result']}")
    if not isinstance(report["upper_commit"], str) or not GIT_SHA.fullmatch(report["upper_commit"]):
        failures.append(f"{name} has no traceable upper commit")
    if not isinstance(report["firmware_commit"], str) or not GIT_SHA.fullmatch(
        report["firmware_commit"]
    ):
        failures.append(f"{name} has no traceable firmware commit")
    if (
        isinstance(report["hardware_revision"], bool)
        or not isinstance(report["hardware_revision"], int)
        or report["hardware_revision"] <= 0
    ):
        failures.append(f"{name} has no positive hardware revision")
    if not isinstance(report["config_sha256"], str) or not SHA256.fullmatch(
        report["config_sha256"]
    ):
        failures.append(f"{name} has no effective config SHA-256")
    artifacts = report["artifact_sha256"]
    if not isinstance(artifacts, dict) or not artifacts:
        failures.append(f"{name} has no generated artifact SHA-256 manifest")
    elif any(
        not isinstance(path, str)
        or not path
        or not isinstance(digest, str)
        or not SHA256.fullmatch(digest)
        for path, digest in artifacts.items()
    ):
        failures.append(f"{name} has an invalid artifact SHA-256 manifest")
    if (
        isinstance(report["parameter_crc32"], bool)
        or not isinstance(report["parameter_crc32"], int)
        or not 0 < report["parameter_crc32"] <= 0xFFFFFFFF
    ):
        failures.append(f"{name} has no measured firmware parameter CRC")
    if not isinstance(report["calibration_version"], str) or not report["calibration_version"]:
        failures.append(f"{name} has no calibration version")
    if not isinstance(report["metrics"], dict) or not report["metrics"]:
        failures.append(f"{name} has no measured metrics")
    return failures


def release_failures(root=ROOT):
    root = Path(root)
    manifest_path = root / "verification" / "configs" / "release" / f"{RELEASE}.yaml"
    try:
        manifest = _mapping(manifest_path)
    except ValueError as exc:
        return [str(exc)]
    failures = []
    for name, relative in manifest.get("release_materials", {}).items():
        if not (root / relative).is_file():
            failures.append(f"missing release material {name}: {relative}")
    reports = {}
    for name, relative in manifest.get("hardware_gates", {}).items():
        path = root / relative
        if not path.is_file():
            failures.append(f"missing hardware report {name}: {relative}")
            continue
        try:
            reports[name] = _mapping(path)
        except ValueError as exc:
            failures.append(str(exc))
            continue
        failures.extend(_validate_result(name, reports[name], manifest["release"]))
    for name in ("calibration_report", "dynamics_report"):
        if name in reports and reports[name].get("status") != "final":
            failures.append(f"{name} is not final")
    compatibility_path = root / manifest.get("release_materials", {}).get(
        "compatibility", "compatibility/firmware.yaml"
    )
    if compatibility_path.is_file():
        compatibility = _mapping(compatibility_path)
        upper = compatibility.get("upper", {})
        tested_upper_commit = upper.get("tested_commit")
        if not isinstance(tested_upper_commit, str) or not GIT_SHA.fullmatch(tested_upper_commit):
            failures.append("upper tested commit is not locked")
        if upper.get("release_candidate_commit") != tested_upper_commit:
            failures.append("upper release candidate differs from tested commit")
        for name, report in reports.items():
            if report.get("result") == "PASS" and report.get("upper_commit") != tested_upper_commit:
                failures.append(f"{name} upper commit differs from tested commit")
        if compatibility.get("release_compatible") is not True:
            failures.append("firmware compatibility is not release-compatible")
        compatible_commit = compatibility.get("firmware", {}).get("compatible_commit")
        tested_firmware_commit = compatibility.get("firmware", {}).get("tested_commit")
        if not isinstance(compatible_commit, str) or not GIT_SHA.fullmatch(compatible_commit):
            failures.append("compatible upper-v3 firmware commit is not locked")
        if tested_firmware_commit != compatible_commit:
            failures.append("tested firmware commit differs from compatibility lock")
        for report in reports.values():
            if (
                report.get("result") == "PASS"
                and report.get("firmware_commit") != compatible_commit
            ):
                failures.append("report firmware commit differs from compatibility lock")
        expected_crc = compatibility.get("firmware", {}).get("parameter_crc32")
        expected_hardware = compatibility.get("firmware", {}).get("hardware_revision")
        passing_reports = [report for report in reports.values() if report.get("result") == "PASS"]
        for report in passing_reports:
            if report.get("parameter_crc32") != expected_crc:
                failures.append("report parameter CRC differs from compatibility lock")
            if report.get("hardware_revision") != expected_hardware:
                failures.append("report hardware revision differs from compatibility lock")
        config_hashes = {report.get("config_sha256") for report in passing_reports}
        calibration_versions = {report.get("calibration_version") for report in passing_reports}
        artifact_manifests = {
            json.dumps(report.get("artifact_sha256"), sort_keys=True) for report in passing_reports
        }
        if len(config_hashes) > 1:
            failures.append("PASS reports do not share one effective config SHA-256")
        if len(calibration_versions) > 1:
            failures.append("PASS reports do not share one calibration version")
        if len(artifact_manifests) > 1:
            failures.append("PASS reports do not share one generated artifact manifest")
    return failures


def main() -> int:
    failures = release_failures()
    if failures:
        print(f"{RELEASE} release blocked:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1
    print(f"{RELEASE} release gate PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
