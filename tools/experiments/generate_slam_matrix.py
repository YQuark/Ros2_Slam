#!/usr/bin/env python3
"""Generate immutable slam_toolbox YAML candidates and a matrix manifest."""

import argparse
import copy
import json
from pathlib import Path

import yaml

from experiment_gate import load_experiment
from latin_hypercube import generate_lhs


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "verification" / "configs" / "experiments" / "slam-sweep-v0.4.0.yaml"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--experiment", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    experiment = load_experiment(args.experiment)
    baseline_path = ROOT / experiment["baseline_params"]
    baseline = yaml.safe_load(baseline_path.read_text(encoding="utf-8"))
    candidates = generate_lhs(
        experiment["parameter_axes"],
        count=int(experiment["coarse_candidate_count"]),
        seed=int(experiment["seed"]),
    )
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": 1,
        "experiment": experiment["name"],
        "training_datasets": experiment["training_datasets"],
        "validation_datasets": experiment["validation_datasets"],
        "validation_is_holdout": True,
        "candidates": [],
    }
    for candidate in candidates:
        generated = copy.deepcopy(baseline)
        parameters = generated["slam_toolbox"]["ros__parameters"]
        parameters.update(candidate["parameters"])
        output_path = args.output / f"{candidate['id']}.yaml"
        output_path.write_text(yaml.safe_dump(generated, sort_keys=False), encoding="utf-8")
        manifest["candidates"].append(
            {
                "id": candidate["id"],
                "params_file": str(output_path),
                "parameters": candidate["parameters"],
            }
        )
    manifest_path = args.output / "matrix.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
