#!/usr/bin/env python3
"""Generate provisional gyro Allan-deviation evidence without changing runtime config."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path
from typing import Iterable

import yaml


def allan_deviation(samples: list[float], sample_period_sec: float) -> list[tuple[float, float]]:
    result = []
    cluster = 1
    while cluster * 4 <= len(samples):
        count = len(samples) // cluster
        means = [
            statistics.fmean(samples[index * cluster : (index + 1) * cluster])
            for index in range(count)
        ]
        variance = 0.5 * statistics.fmean(
            (means[index + 1] - means[index]) ** 2 for index in range(len(means) - 1)
        )
        result.append((cluster * sample_period_sec, math.sqrt(max(variance, 0.0))))
        cluster *= 2
    return result


def analyze(rows: Iterable[dict[str, str]]) -> dict:
    parsed = []
    for row in rows:
        timestamp = float(row["timestamp_sec"])
        gyro = float(row["gyro_z_radps"])
        temperature = float(row.get("temperature_c", "nan"))
        if not math.isfinite(timestamp) or not math.isfinite(gyro):
            raise ValueError("finite timestamp_sec and gyro_z_radps are required")
        parsed.append((timestamp, gyro, temperature))
    if len(parsed) < 128:
        raise ValueError("at least 128 stationary samples are required")
    periods = [parsed[index + 1][0] - parsed[index][0] for index in range(len(parsed) - 1)]
    if any(period <= 0.0 for period in periods):
        raise ValueError("timestamps must be strictly increasing")
    period = statistics.median(periods)
    max_relative_period_error = max(abs(value - period) for value in periods) / period
    if max_relative_period_error > 0.05:
        raise ValueError("sample period jitter exceeds 5%; resample before Allan analysis")
    curve = allan_deviation([row[1] for row in parsed], period)
    minimum_tau, minimum_deviation = min(curve, key=lambda item: item[1])
    first_tau, first_deviation = curve[0]
    last_tau, last_deviation = curve[-1]
    temperatures = [row[2] for row in parsed if math.isfinite(row[2])]
    temperature_span = max(temperatures) - min(temperatures) if temperatures else None
    return {
        "schema_version": 1,
        "status": "provisional",
        "sample_count": len(parsed),
        "sample_rate_hz": 1.0 / period,
        "max_relative_period_error": max_relative_period_error,
        "allan_curve": [{"tau_sec": tau, "deviation_radps": deviation} for tau, deviation in curve],
        "estimates": {
            "angle_random_walk_rad_sqrt_s": first_deviation * math.sqrt(first_tau),
            "bias_instability_radps": minimum_deviation / 0.664,
            "rate_random_walk_rad_s_sqrt_s": last_deviation / math.sqrt(last_tau),
            "bias_minimum_tau_sec": minimum_tau,
            "recommended_gyro_variance": minimum_deviation**2,
            "temperature_span_c": temperature_span,
        },
        "promotion": "manual review plus independent cold/hot/vibration runs required",
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
