#!/usr/bin/env python3
"""Fail closed until every v0.3.0 software and physical release gate has evidence."""

from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "config" / "release" / "v0.3.0.yaml"


def release_failures(root=ROOT):
    root = Path(root)
    manifest = yaml.safe_load(
        (root / "config" / "release" / "v0.3.0.yaml").read_text(encoding="utf-8")
    )
    failures = []
    for name, relative in manifest["release_materials"].items():
        if not (root / relative).is_file():
            failures.append(f"missing release material {name}: {relative}")
    for report_name in ("hil_report", "vehicle_report"):
        relative = manifest["hardware_gates"][report_name]
        text = (root / relative).read_text(encoding="utf-8")
        if "Overall status: PASS" not in text:
            failures.append(f"{report_name} is not PASS: {relative}")
    calibration = yaml.safe_load(
        (root / manifest["hardware_gates"]["calibration"]).read_text(encoding="utf-8")
    )
    if calibration.get("status") != "final":
        failures.append("robot calibration is not final")
    dynamics = yaml.safe_load(
        (root / manifest["hardware_gates"]["vehicle_dynamics"]).read_text(encoding="utf-8")
    )
    if dynamics.get("status") != "completed":
        failures.append("vehicle dynamics are not completed")
    return failures


def main() -> int:
    failures = release_failures()
    if failures:
        print("v0.3.0 release blocked:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1
    print("v0.3.0 release gate PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
