#!/usr/bin/env python3
"""Enforce coverage thresholds for the safety-critical upper modules."""

import argparse
import json
from pathlib import Path


THRESHOLDS = {
    "control_policy.py": 90.0,
    "protocol_v3.py": 90.0,
    "odometry.py": 90.0,
    "bridge_core.py": 85.0,
}
TOTAL_THRESHOLD = 75.0


def coverage_failures(data):
    failures = []
    files = data.get("files", {})
    for suffix, threshold in THRESHOLDS.items():
        matches = [details for name, details in files.items() if Path(name).name == suffix]
        if len(matches) != 1:
            failures.append(f"{suffix}: expected one coverage entry, got {len(matches)}")
            continue
        actual = float(matches[0]["summary"]["percent_covered"])
        if actual < threshold:
            failures.append(f"{suffix}: {actual:.2f}% < {threshold:.2f}%")
    total = float(data.get("totals", {}).get("percent_covered", 0.0))
    if total < TOTAL_THRESHOLD:
        failures.append(f"TOTAL: {total:.2f}% < {TOTAL_THRESHOLD:.2f}%")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("coverage_json", type=Path, nargs="?", default=Path("coverage.json"))
    args = parser.parse_args()
    failures = coverage_failures(json.loads(args.coverage_json.read_text(encoding="utf-8")))
    if failures:
        print("Coverage gate failed:")
        for failure in failures:
            print(f"  {failure}")
        return 1
    print("Coverage gate OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
