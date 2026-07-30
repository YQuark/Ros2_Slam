#!/usr/bin/env python3
"""Evaluate labelled motion-consistency risk without calling it probability."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path
from typing import Iterable

import yaml


def analyze(rows: Iterable[dict[str, str]]) -> dict:
    parsed = []
    for row in rows:
        label = int(row["fault_label"])
        risk = float(row["motion_consistency_risk"])
        if label not in (0, 1) or not math.isfinite(risk) or not 0.0 <= risk <= 1.0:
            raise ValueError("fault_label must be binary and risk must be finite in [0,1]")
        parsed.append((str(row["scenario"]), label, risk))
    if not parsed or {row[1] for row in parsed} != {0, 1}:
        raise ValueError("both normal and fault-labelled samples are required")

    positives = sum(row[1] for row in parsed)
    negatives = len(parsed) - positives
    roc = []
    thresholds = sorted({0.0, math.nextafter(1.0, math.inf), *(row[2] for row in parsed)})
    for threshold in thresholds:
        true_positive = sum(label == 1 and risk >= threshold for _, label, risk in parsed)
        false_positive = sum(label == 0 and risk >= threshold for _, label, risk in parsed)
        tpr = true_positive / positives
        fpr = false_positive / negatives
        roc.append({"threshold": threshold, "tpr": tpr, "fpr": fpr, "fnr": 1.0 - tpr})
    deployable = [item for item in roc if item["threshold"] <= 1.0]
    selected = max(deployable, key=lambda item: (item["tpr"] - item["fpr"], -item["threshold"]))
    ordered_curve = sorted(roc, key=lambda item: (item["fpr"], item["tpr"]))
    auc = sum(
        (right["fpr"] - left["fpr"]) * (right["tpr"] + left["tpr"]) * 0.5
        for left, right in zip(ordered_curve, ordered_curve[1:])
    )
    scenarios = {}
    for scenario in sorted({row[0] for row in parsed}):
        values = [row[2] for row in parsed if row[0] == scenario]
        labels = [row[1] for row in parsed if row[0] == scenario]
        scenarios[scenario] = {
            "sample_count": len(values),
            "fault_fraction": statistics.fmean(labels),
            "risk_mean": statistics.fmean(values),
            "risk_max": max(values),
        }
    return {
        "schema_version": 1,
        "status": "provisional",
        "sample_count": len(parsed),
        "scenario_count": len({row[0] for row in parsed}),
        "roc": roc,
        "roc_auc": auc,
        "scenario_distributions": scenarios,
        "candidate_threshold": selected,
        "terminology": "motion_consistency_risk is not a calibrated probability",
        "promotion": "manual independent validation required",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with args.input.open(newline="", encoding="utf-8") as stream:
        report = analyze(csv.DictReader(stream))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
