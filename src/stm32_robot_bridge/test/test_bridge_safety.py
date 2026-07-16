import math

import pytest

from stm32_robot_bridge.bridge_state import BridgeState, BridgeStateMachine
from stm32_robot_bridge.command_guard import CommandGuard, CommandRejected


def make_guard() -> CommandGuard:
    return CommandGuard(
        hard_max_linear_mps=0.45,
        hard_max_angular_radps=1.50,
        max_command_age_sec=0.15,
        status_timeout_sec=0.25,
    )


def test_guard_rejects_nan_inf_stale_future_and_missing_status():
    guard = make_guard()
    valid = dict(
        command_stamp_sec=10.0,
        now_sec=10.1,
        status_age_sec=0.1,
        drive_permitted=True,
        status_flags=0,
        protocol_version=2,
    )

    for vx, wz in ((math.nan, 0.0), (0.0, math.inf), (-math.inf, 0.0)):
        with pytest.raises(CommandRejected):
            guard.validate(vx=vx, wz=wz, **valid)
    with pytest.raises(CommandRejected, match="command is stale"):
        guard.validate(vx=0.1, wz=0.0, **{**valid, "now_sec": 10.2})
    with pytest.raises(CommandRejected, match="future"):
        guard.validate(vx=0.1, wz=0.0, **{**valid, "now_sec": 9.9})
    with pytest.raises(CommandRejected, match="STATUS unavailable"):
        guard.validate(vx=0.1, wz=0.0, **{**valid, "status_age_sec": None})


def test_guard_hard_clamps_only_after_all_drive_checks_pass():
    guard = make_guard()
    accepted = guard.validate(
        vx=100.0,
        wz=-100.0,
        command_stamp_sec=10.0,
        now_sec=10.1,
        status_age_sec=0.1,
        drive_permitted=True,
        status_flags=0,
        protocol_version=2,
    )

    assert accepted.vx == 0.45
    assert accepted.wz == -1.50

    for overrides, reason in (
        ({"drive_permitted": False}, "state"),
        ({"status_flags": 0x01}, "fault"),
        ({"protocol_version": 1}, "protocol"),
        ({"status_age_sec": 0.26}, "STATUS stale"),
    ):
        values = dict(
            vx=0.1,
            wz=0.0,
            command_stamp_sec=10.0,
            now_sec=10.1,
            status_age_sec=0.1,
            drive_permitted=True,
            status_flags=0,
            protocol_version=2,
        )
        values.update(overrides)
        with pytest.raises(CommandRejected, match=reason):
            guard.validate(**values)


def test_state_machine_requires_status_before_drive_and_releases_on_timeout():
    machine = BridgeStateMachine()
    assert machine.state is BridgeState.DISCONNECTED
    assert machine.can_drive is False

    machine.on_serial_opened()
    assert machine.state is BridgeState.SETTLING
    machine.on_settled()
    assert machine.state is BridgeState.WAIT_STATUS
    machine.on_valid_status(status_flags=0)
    assert machine.state is BridgeState.READY
    assert machine.can_drive is True

    machine.on_drive_enabled()
    assert machine.state is BridgeState.ACTIVE
    machine.on_status_timeout()
    assert machine.state is BridgeState.WAIT_STATUS
    assert machine.can_drive is False


def test_fault_recovery_requires_disable_then_enable_edge():
    machine = BridgeStateMachine()
    machine.on_serial_opened()
    machine.on_settled()
    machine.on_valid_status(status_flags=0)
    machine.on_drive_enabled()

    machine.on_valid_status(status_flags=0x02)
    assert machine.state is BridgeState.FAULT
    machine.on_valid_status(status_flags=0)
    assert machine.state is BridgeState.FAULT
    assert machine.can_drive is False

    machine.on_disable_command()
    assert machine.state is BridgeState.READY
    assert machine.can_drive is True
    machine.on_drive_enabled()
    assert machine.state is BridgeState.ACTIVE


def test_disconnect_always_returns_to_disconnected():
    machine = BridgeStateMachine()
    machine.on_serial_opened()
    machine.on_settled()
    machine.on_valid_status(status_flags=0)

    machine.on_disconnected()

    assert machine.state is BridgeState.DISCONNECTED
    assert machine.can_drive is False
