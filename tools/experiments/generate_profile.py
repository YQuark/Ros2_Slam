#!/usr/bin/env python3
"""Deprecated v0.4.x wrapper for verification/runners/experiments."""

import runpy
from pathlib import Path

_TARGET = (
    Path(__file__).resolve().parents[2] / "verification/runners/experiments/generate_profile.py"
)
_NAMESPACE = runpy.run_path(str(_TARGET))
merge = _NAMESPACE["merge"]
generate = _NAMESPACE["generate"]
main = _NAMESPACE["main"]

if __name__ == "__main__":
    raise SystemExit(main())
