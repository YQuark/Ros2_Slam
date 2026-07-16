from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "robot_control"))

from robot_control.control_policy import (
    Command,
    CommandMux,
    InvalidCommandError,
    MotionLimiter,
    SourceConfig,
)


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
    assert active.active is True
    assert active.command == Command(linear_x=0.2, angular_z=0.0)
    assert expired.source == "idle"
    assert expired.active is False
    assert expired.command == Command(linear_x=0.0, angular_z=0.0)


def test_mux_rejects_non_finite_and_clears_only_that_source() -> None:
    mux = CommandMux(
        source_config=SourceConfig(),
        linear_limit=0.4,
        angular_limit=1.5,
        timeout_sec=0.25,
    )
    mux.update("nav", Command(0.2, 0.1), now_sec=1.0)
    mux.update("teleop", Command(0.3, 0.2), now_sec=1.0)

    for command in (
        Command(float("nan"), 0.0),
        Command(0.0, float("nan")),
        Command(float("inf"), 0.0),
        Command(0.0, float("-inf")),
    ):
        with pytest.raises(InvalidCommandError):
            mux.update("teleop", command, now_sec=1.1)

    selected = mux.select(now_sec=1.1)
    assert selected.source == "nav"
    assert selected.command == Command(0.2, 0.1)
    assert mux.reject_count == 4


def test_mux_rejects_implausible_absolute_input_before_soft_clamp() -> None:
    mux = CommandMux(
        source_config=SourceConfig(),
        linear_limit=0.4,
        angular_limit=1.5,
        timeout_sec=0.25,
        input_linear_abs_max=5.0,
        input_angular_abs_max=20.0,
    )

    with pytest.raises(InvalidCommandError):
        mux.update("teleop", Command(5.1, 0.0), now_sec=1.0)
    with pytest.raises(InvalidCommandError):
        mux.update("teleop", Command(0.0, 20.1), now_sec=1.0)

    assert mux.select(1.0).active is False


def test_motion_limiter_applies_acceleration_and_jerk_limits():
    limiter = MotionLimiter(
        max_linear_accel=1.0,
        max_angular_accel=2.0,
        max_linear_jerk=2.0,
        max_angular_jerk=4.0,
        max_dt_sec=0.1,
    )

    assert limiter.limit(Command(1.0, 1.0), now_sec=0.0) == Command(0.0, 0.0)
    first = limiter.limit(Command(1.0, 1.0), now_sec=0.1)
    second = limiter.limit(Command(1.0, 1.0), now_sec=0.2)

    assert first.linear_x == pytest.approx(0.02)
    assert first.angular_z == pytest.approx(0.04)
    assert second.linear_x == pytest.approx(0.06)
    assert second.angular_z == pytest.approx(0.12)


def test_motion_limiter_is_continuous_across_source_switch_and_resets_on_release():
    limiter = MotionLimiter(
        max_linear_accel=1.0,
        max_angular_accel=1.0,
        max_linear_jerk=10.0,
        max_angular_jerk=10.0,
        max_dt_sec=0.1,
    )
    limiter.limit(Command(1.0, 0.0), now_sec=0.0)
    before_switch = limiter.limit(Command(1.0, 0.0), now_sec=0.1)
    after_switch = limiter.limit(Command(-1.0, 0.0), now_sec=0.2)

    assert before_switch.linear_x > 0.0
    assert after_switch.linear_x >= 0.0

    limiter.reset()
    assert limiter.current == Command(0.0, 0.0)
