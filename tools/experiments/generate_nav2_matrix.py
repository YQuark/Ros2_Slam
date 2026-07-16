#!/usr/bin/env python3
"""Generate one-factor Nav2 parameter candidates after vehicle characterization."""

import argparse
import copy
import json
from pathlib import Path

import yaml

from experiment_gate import generate_one_factor_candidates
from navigation_gate import load_contract


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = ROOT / "config" / "experiments" / "nav2_acceptance.yaml"


def set_path(tree, path, value):
    target = tree
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    contract = load_contract(args.contract)
    missing_measurements = [
        name
        for name, value in contract["vehicle_characterization"]["required_measurements"].items()
        if value is None
    ]
    if missing_measurements:
        raise SystemExit("vehicle characterization incomplete: " + ", ".join(missing_measurements))
    baseline = yaml.safe_load((ROOT / contract["baseline_params"]).read_text(encoding="utf-8"))
    candidates = generate_one_factor_candidates(contract)
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {"schema_version": 1, "experiment": contract["name"], "candidates": []}
    for candidate in candidates:
        generated = copy.deepcopy(baseline)
        for parameter, value in candidate["parameters"].items():
            for path in contract["parameter_paths"][parameter]:
                set_path(generated, path, value)
        path = args.output / f"{candidate['id']}.yaml"
        path.write_text(yaml.safe_dump(generated, sort_keys=False), encoding="utf-8")
        manifest["candidates"].append({"id": candidate["id"], "params_file": str(path)})
    manifest_path = args.output / "matrix.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
