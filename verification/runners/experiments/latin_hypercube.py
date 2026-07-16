#!/usr/bin/env python3
"""Dependency-free deterministic Latin Hypercube candidate generator."""

from __future__ import annotations

import random
from typing import Mapping, Sequence


def generate_lhs(axes: Mapping[str, Sequence[float]], count: int = 32, seed: int = 42):
    rng = random.Random(seed)
    names = tuple(axes)
    columns = {}
    for name in names:
        values = tuple(float(value) for value in axes[name])
        if len(values) < 2:
            raise ValueError(f"axis {name} needs a range")
        low, high = min(values), max(values)
        strata = [(index + rng.random()) / count for index in range(count)]
        rng.shuffle(strata)
        columns[name] = [low + fraction * (high - low) for fraction in strata]
    return [
        {
            "id": f"lhs_{index + 1:02d}",
            "parameters": {name: columns[name][index] for name in names},
        }
        for index in range(count)
    ]
