"""Shared planning and validation logic for the versioned rosbag benchmark set."""

from pathlib import Path
from typing import Any, Dict, NamedTuple, Tuple

import yaml


class RecordPlan(NamedTuple):
    dataset: str
    output_dir: Path
    command: Tuple[str, ...]


class ValidationResult(NamedTuple):
    valid: bool
    duration_sec: float
    minimum_duration_sec: float
    duration_ok: bool
    missing_topics: Tuple[str, ...]
    empty_topics: Tuple[str, ...]


def load_catalog(path: Path) -> Dict[str, Any]:
    catalog = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(catalog, dict) or catalog.get("schema_version") != 1:
        raise ValueError("unsupported rosbag dataset catalog")
    if not catalog.get("required_topics") or not catalog.get("datasets"):
        raise ValueError("catalog must define required_topics and datasets")
    return catalog


def build_record_plan(
    catalog: Dict[str, Any],
    dataset: str,
    *,
    output_root: Path,
    run_id: str,
) -> RecordPlan:
    if dataset not in catalog["datasets"]:
        raise ValueError(f"unknown dataset: {dataset}")
    if not run_id or "/" in run_id or run_id in (".", ".."):
        raise ValueError("run_id must be a non-empty path component")
    output_dir = Path(output_root) / dataset / run_id
    command = (
        "ros2",
        "bag",
        "record",
        "--output",
        str(output_dir),
        "--storage",
        str(catalog.get("storage_id", "sqlite3")),
        *tuple(str(topic) for topic in catalog["required_topics"]),
    )
    return RecordPlan(dataset, output_dir, command)


def validate_metadata(
    catalog: Dict[str, Any],
    dataset: str,
    metadata: Dict[str, Any],
) -> ValidationResult:
    if dataset not in catalog["datasets"]:
        raise ValueError(f"unknown dataset: {dataset}")
    info = metadata.get("rosbag2_bagfile_information", {})
    duration_sec = float(info.get("duration", {}).get("nanoseconds", 0)) / 1_000_000_000.0
    minimum_duration_sec = float(catalog["datasets"][dataset]["minimum_duration_sec"])
    message_counts = {}
    for item in info.get("topics_with_message_count", []):
        topic = item.get("topic_metadata", {}).get("name")
        if topic:
            message_counts[str(topic)] = int(item.get("message_count", 0))

    required = tuple(str(topic) for topic in catalog["required_topics"])
    missing = tuple(topic for topic in required if topic not in message_counts)
    empty = tuple(
        topic for topic in required if message_counts.get(topic, 0) == 0 and topic not in missing
    )
    duration_ok = duration_sec >= minimum_duration_sec
    return ValidationResult(
        valid=duration_ok and not missing and not empty,
        duration_sec=duration_sec,
        minimum_duration_sec=minimum_duration_sec,
        duration_ok=duration_ok,
        missing_topics=missing,
        empty_topics=empty,
    )
