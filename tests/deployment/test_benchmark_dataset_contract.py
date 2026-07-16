import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "config" / "benchmarks" / "rosbag_datasets.yaml"
MODULE_PATH = ROOT / "tools" / "datasets" / "benchmark_bags.py"


def load_module():
    spec = importlib.util.spec_from_file_location("benchmark_bags", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_dataset_catalog_covers_all_seven_benchmarks_and_required_topics():
    catalog = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

    assert catalog["schema_version"] == 1
    assert list(catalog["datasets"]) == [
        "bag_01_static",
        "bag_02_straight",
        "bag_03_rotation",
        "bag_04_square",
        "bag_05_figure8",
        "bag_06_mapping",
        "bag_07_navigation",
    ]
    assert set(catalog["required_topics"]) == {
        "/scan",
        "/wheel/odom",
        "/imu/data",
        "/odom",
        "/tf",
        "/tf_static",
        "/cmd_vel/nav",
        "/chassis/command",
        "/diagnostics",
    }
    for dataset in catalog["datasets"].values():
        assert dataset["minimum_duration_sec"] > 0
        assert dataset["procedure"]


def test_record_plan_is_deterministic_and_rejects_unknown_dataset(tmp_path):
    module = load_module()
    catalog = module.load_catalog(CONFIG_PATH)

    plan = module.build_record_plan(
        catalog,
        "bag_04_square",
        output_root=tmp_path,
        run_id="run-001",
    )

    assert plan.output_dir == tmp_path / "bag_04_square" / "run-001"
    assert plan.command[:4] == ("ros2", "bag", "record", "--output")
    assert tuple(catalog["required_topics"]) == plan.command[-9:]
    with pytest.raises(ValueError, match="unknown dataset"):
        module.build_record_plan(catalog, "bag_99", output_root=tmp_path, run_id="x")


def test_metadata_validation_reports_missing_topics_duration_and_empty_streams():
    module = load_module()
    catalog = module.load_catalog(CONFIG_PATH)
    metadata = {
        "rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 5_000_000_000},
            "topics_with_message_count": [
                {
                    "topic_metadata": {"name": topic},
                    "message_count": 10 if topic != "/imu/data" else 0,
                }
                for topic in catalog["required_topics"]
                if topic != "/diagnostics"
            ],
        }
    }

    result = module.validate_metadata(catalog, "bag_01_static", metadata)

    assert result.valid is False
    assert "/diagnostics" in result.missing_topics
    assert "/imu/data" in result.empty_topics
    assert result.duration_ok is False
