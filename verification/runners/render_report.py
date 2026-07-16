#!/usr/bin/env python3
"""Render a machine result as a small read-only Markdown summary."""

import argparse
from pathlib import Path

import yaml


def render(data: dict) -> str:
    lines = [f"# {data['release']} verification", "", f"Overall status: {data['result']}", ""]
    for key in (
        "upper_commit",
        "firmware_commit",
        "hardware_revision",
        "config_sha256",
        "started_at",
    ):
        lines.append(f"- {key}: {data.get(key)}")
    lines.extend(("", "## Metrics", ""))
    metrics = data.get("metrics") or {}
    lines.extend(f"- {key}: {value}" for key, value in sorted(metrics.items()))
    if data.get("blocker"):
        lines.extend(("", f"Blocker: {data['blocker']}"))
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    data = yaml.safe_load(args.result.read_text(encoding="utf-8"))
    args.output.write_text(render(data), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
