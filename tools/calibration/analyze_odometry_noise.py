#!/usr/bin/env python3
"""Fit provisional differential-drive increment noise from labelled CSV trials."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Iterable

import yaml


REQUIRED_FIELDS = {"condition", "distance_m", "left_error_m", "right_error_m"}


def _p95(values: list[float]) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    return ordered[min(math.ceil(0.95 * len(ordered)) - 1, len(ordered) - 1)]


def _fit_variance(distance: list[float], error: list[float]) -> tuple[float, float]:
    targets = [value * value for value in error]
    mean_x, mean_y = statistics.fmean(distance), statistics.fmean(targets)
    denominator = sum((value - mean_x) ** 2 for value in distance)
    slope = (
        sum((x - mean_x) * (y - mean_y) for x, y in zip(distance, targets)) / denominator
        if denominator > 0.0
        else 0.0
    )
    return max(mean_y - slope * mean_x, 0.0), max(slope, 0.0)


def _detrend(values: list[float], errors: list[float]) -> tuple[list[float], float, float]:
    """Separate deterministic scale/bias error from stochastic residuals."""

    mean_x, mean_error = statistics.fmean(values), statistics.fmean(errors)
    denominator = sum((value - mean_x) ** 2 for value in values)
    slope = (
        sum((x - mean_x) * (error - mean_error) for x, error in zip(values, errors)) / denominator
        if denominator > 0.0
        else 0.0
    )
    intercept = mean_error - slope * mean_x
    return (
        [error - (intercept + slope * value) for value, error in zip(values, errors)],
        intercept,
        slope,
    )


def analyze(rows: Iterable[dict[str, str]]) -> dict:
    parsed = []
    for row in rows:
        if not REQUIRED_FIELDS.issubset(row):
            raise ValueError(f"required CSV fields: {sorted(REQUIRED_FIELDS)}")
        values = (
            str(row["condition"]),
            float(row["distance_m"]),
            float(row["left_error_m"]),
            float(row["right_error_m"]),
        )
        if not values[0] or not all(math.isfinite(value) for value in values[1:]):
            raise ValueError("condition and finite numeric measurements are required")
        parsed.append(values)
    if len(parsed) < 3:
        raise ValueError("at least three trials are required")

    signed_distance = [row[1] for row in parsed]
    distance = [abs(value) for value in signed_distance]
    left = [row[2] for row in parsed]
    right = [row[3] for row in parsed]
    left_residual, left_bias, left_scale_error = _detrend(signed_distance, left)
    right_residual, right_bias, right_scale_error = _detrend(signed_distance, right)
    left_floor, left_per_meter = _fit_variance(distance, left_residual)
    right_floor, right_per_meter = _fit_variance(distance, right_residual)
    mean_left, mean_right = statistics.fmean(left), statistics.fmean(right)
    mean_left_residual = statistics.fmean(left_residual)
    mean_right_residual = statistics.fmean(right_residual)
    covariance = sum(
        (lvalue - mean_left_residual) * (rvalue - mean_right_residual)
        for lvalue, rvalue in zip(left_residual, right_residual)
    ) / max(len(parsed) - 1, 1)
    left_std = statistics.stdev(left)
    right_std = statistics.stdev(right)
    left_residual_std = statistics.stdev(left_residual)
    right_residual_std = statistics.stdev(right_residual)
    correlation = (
        covariance / (left_residual_std * right_residual_std)
        if left_residual_std and right_residual_std
        else 0.0
    )

    grouped: dict[str, list[float]] = defaultdict(list)
    for condition, _, left_error, right_error in parsed:
        grouped[condition].append(0.5 * (abs(left_error) + abs(right_error)))
    return {
        "schema_version": 1,
        "status": "provisional",
        "trial_count": len(parsed),
        "metrics": {
            "left_mean_error_m": mean_left,
            "right_mean_error_m": mean_right,
            "left_std_m": left_std,
            "right_std_m": right_std,
            "left_p95_abs_error_m": _p95([abs(value) for value in left]),
            "right_p95_abs_error_m": _p95([abs(value) for value in right]),
            "left_right_residual_correlation": correlation,
            "left_bias_m": left_bias,
            "right_bias_m": right_bias,
            "left_scale_error_per_meter": left_scale_error,
            "right_scale_error_per_meter": right_scale_error,
            "condition_p95_abs_error_m": {
                name: _p95(values) for name, values in sorted(grouped.items())
            },
        },
        "candidate_odometry_noise": {
            "left_variance_floor_m2": left_floor,
            "right_variance_floor_m2": right_floor,
            "left_variance_per_meter": left_per_meter,
            "right_variance_per_meter": right_per_meter,
            "left_right_correlation": max(min(correlation, 0.99), -0.99),
        },
        "promotion": "apply geometry scale calibration, then validate residual noise independently",
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
