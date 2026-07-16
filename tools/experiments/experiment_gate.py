"""Pure experiment-matrix and strict acceptance-gate helpers."""

import math
from pathlib import Path
from typing import Any, Dict, List, NamedTuple, Tuple

import yaml


class GateResult(NamedTuple):
    passed: bool
    missing: Tuple[str, ...]
    failed: Tuple[str, ...]


def load_experiment(path: Path) -> Dict[str, Any]:
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(config, dict) or config.get("schema_version") != 1:
        raise ValueError("unsupported experiment schema")
    if not config.get("parameter_axes") or not config.get("acceptance"):
        raise ValueError("experiment must define parameter_axes and acceptance")
    for parameter, values in config["parameter_axes"].items():
        if not isinstance(values, list) or not values:
            raise ValueError(f"parameter axis is empty: {parameter}")
    return config


def _value_id(value: Any) -> str:
    return str(value).replace("-", "m").replace(".", "p")


def generate_one_factor_candidates(config: Dict[str, Any]) -> List[Dict[str, Any]]:
    baseline = {name: values[0] for name, values in config["parameter_axes"].items()}
    candidates = [{"id": "baseline", "parameters": baseline}]
    for name, values in config["parameter_axes"].items():
        for value in values[1:]:
            parameters = dict(baseline)
            parameters[name] = value
            candidates.append(
                {
                    "id": f"{name}-{_value_id(value)}",
                    "parameters": parameters,
                }
            )
    return candidates


def evaluate_metrics(config: Dict[str, Any], metrics: Dict[str, Any]) -> GateResult:
    missing = tuple(
        name for name in config["acceptance"] if name not in metrics or metrics[name] is None
    )
    failed = []
    for name, rule in config["acceptance"].items():
        if name in missing:
            continue
        value = float(metrics[name])
        limit = float(rule["limit"])
        operator = rule["operator"]
        if not math.isfinite(value):
            passed = False
        elif operator == "<=":
            passed = value <= limit
        elif operator == ">=":
            passed = value >= limit
        else:
            raise ValueError(f"unsupported operator for {name}: {operator}")
        if not passed:
            failed.append(name)
    failed_tuple = tuple(failed)
    return GateResult(not missing and not failed_tuple, missing, failed_tuple)
