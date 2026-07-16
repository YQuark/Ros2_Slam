#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from robot_config.compiler import compile_effective_config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--robot-id", default="robot_001")
    parser.add_argument(
        "--profile", choices=("mapping", "navigation", "diagnostics"), default="mapping"
    )
    args = parser.parse_args()
    paths = compile_effective_config(
        args.config_root, args.output_dir, robot_id=args.robot_id, profile=args.profile
    )
    print(paths["manifest"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
