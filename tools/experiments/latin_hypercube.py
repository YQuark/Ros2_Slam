#!/usr/bin/env python3
"""Deprecated v0.4.x wrapper for verification/runners/experiments."""

import runpy
from pathlib import Path

_TARGET = (
    Path(__file__).resolve().parents[2] / "verification/runners/experiments/latin_hypercube.py"
)
_NAMESPACE = runpy.run_path(str(_TARGET))
generate_lhs = _NAMESPACE["generate_lhs"]
