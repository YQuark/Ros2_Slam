#!/usr/bin/env python3
"""Fail-closed v0.5.0-rc1 release gate over machine-readable evidence."""

from __future__ import annotations

import re
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
RELEASE = "v0.5.0-rc1"
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
    if report["hardware_revision"] in (None, ""):
        failures.append(f"{name} has no hardware revision")
    if not isinstance(report["config_sha256"], str) or not SHA256.fullmatch(
        report["config_sha256"]
    ):
        failures.append(f"{name} has no effective config SHA-256")
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
        if compatibility.get("release_compatible") is not True:
            failures.append("firmware compatibility is not release-compatible")
        compatible_commit = compatibility.get("firmware", {}).get("compatible_commit")
        if not isinstance(compatible_commit, str) or not GIT_SHA.fullmatch(compatible_commit):
            failures.append("compatible upper-v3 firmware commit is not locked")
        for report in reports.values():
            if (
                report.get("result") == "PASS"
                and report.get("firmware_commit") != compatible_commit
            ):
                failures.append("report firmware commit differs from compatibility lock")
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
