from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "robot_control"))

from robot_control.control_policy import Command, CommandMux, SourceConfig


def test_source_config_exposes_only_whitelisted_research_topics() -> None:
    sources = SourceConfig(research_sources=("avoidance", "mpc")).topic_map()

    assert sources["teleop"] == "/cmd_vel/teleop"
    assert sources["nav"] == "/cmd_vel/nav"
    assert sources["test"] == "/cmd_vel/test"
    assert sources["research/avoidance"] == "/cmd_vel/research/avoidance"
    assert sources["research/mpc"] == "/cmd_vel/research/mpc"
    assert "/cmd_vel/research/unknown" not in sources.values()


def test_mux_prefers_manual_sources_and_clamps_output() -> None:
    mux = CommandMux(
        source_config=SourceConfig(research_sources=("avoidance",)),
        linear_limit=0.4,
        angular_limit=1.5,
        timeout_sec=0.25,
    )

    mux.update("nav", Command(linear_x=0.2, angular_z=0.1), now_sec=1.00)
    mux.update("research/avoidance", Command(linear_x=0.3, angular_z=0.2), now_sec=1.01)
    mux.update("teleop", Command(linear_x=2.0, angular_z=-3.0), now_sec=1.02)

    selected = mux.select(now_sec=1.10)

    assert selected.source == "teleop"
    assert selected.command == Command(linear_x=0.4, angular_z=-1.5)


def test_mux_drops_expired_sources_and_returns_stop_when_none_active() -> None:
    mux = CommandMux(
        source_config=SourceConfig(research_sources=("avoidance",)),
        linear_limit=0.4,
        angular_limit=1.5,
        timeout_sec=0.25,
    )

    mux.update("nav", Command(linear_x=0.2, angular_z=0.0), now_sec=1.00)
    active = mux.select(now_sec=1.10)
    expired = mux.select(now_sec=1.30)

    assert active.source == "nav"
    assert active.command == Command(linear_x=0.2, angular_z=0.0)
    assert expired.source == "idle"
    assert expired.command == Command(linear_x=0.0, angular_z=0.0)
