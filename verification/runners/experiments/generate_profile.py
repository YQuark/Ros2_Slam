#!/usr/bin/env python3
"""Deep-merge one SLAM/Nav2 base file with a small versioned overlay."""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path

import yaml


def merge(base, overlay):
    result = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def generate(base_path: Path, profile_path: Path, output_path: Path):
    base = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    output = merge(base, profile.get("overlay", {}))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(yaml.safe_dump(output, sort_keys=False), encoding="utf-8")
    return profile["profile"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    metadata = generate(args.base, args.profile, args.output)
    print(f"{metadata['name']} {metadata['status']} -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
