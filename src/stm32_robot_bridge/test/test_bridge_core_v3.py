import pytest
from pathlib import Path

from stm32_robot_bridge.bridge_core import (
    BridgeCore,
    CommandDisposition,
    StatusDisposition,
)
from stm32_robot_bridge.bridge_state import BridgeState
from stm32_robot_bridge.protocol_v3 import (
    ACK_APPLIED,
    ACK_RECEIVED,
    ACK_SESSION_VALID,
    HelloPayload,
    StatusPayload,
)
from stm32_robot_bridge.transport_supervisor import SerialTransportSupervisor


def status(sequence=1, session=7, received=1, applied=1, flags=0):
    return StatusPayload(
        3,
        flags,
        2,
        0x0F,
        0,
        0,
        12.0,
        (0.0,) * 4,
        (0,) * 4,
        (0.0,) * 4,
        (0.0,) * 4,
        (0,) * 4,
        0x0F,
        0,
        0,
        sequence,
        sequence * 50,
        session,
        received,
        applied,
        0,
        0,
        ACK_SESSION_VALID | ACK_RECEIVED | ACK_APPLIED,
    )


def ready_core():
    core = BridgeCore(
        hard_max_linear_mps=0.45,
        hard_max_angular_radps=1.5,
        command_timeout_sec=0.15,
        status_timeout_sec=0.25,
        max_command_age_sec=0.15,
    )
    core.on_connected(7)
    assert core.on_hello(HelloPayload(3, 1, 0x1F, "11" * 20, 1, 2))
    assert core.on_status(status(received=0, applied=0), 1.0)
    assert core.snapshot.state is BridgeState.WAIT_STATUS
    core.on_disable_sent(1)
    assert core.on_status(status(sequence=2), 1.01)
    assert core.snapshot.state is BridgeState.WIRE_REARM_READY
    core.accept_command(
        vx=0,
        wz=0,
        enable=False,
        source=0,
        session_id=99,
        sequence=1,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.01,
    )
    return core


def test_connection_generation_always_requires_upper_layer_disable():
    core = ready_core()
    assert not core.snapshot.rearm_required
    core.on_disconnected()
    core.on_connected(8)
    assert core.snapshot.rearm_required
    assert core.snapshot.rearm_reason_flags


def test_bridge_core_rejects_missing_capability_and_duplicate_status():
    core = BridgeCore(
        hard_max_linear_mps=1,
        hard_max_angular_radps=1,
        command_timeout_sec=0.1,
        status_timeout_sec=0.2,
        max_command_age_sec=0.1,
    )
    core.on_connected(7)
    assert not core.on_hello(HelloPayload(3, 1, 0x0F, "00" * 20, 1, 0))
    assert core.on_hello(HelloPayload(3, 1, 0x1F, "00" * 20, 1, 0))
    assert core.on_status(status(), 1.0)
    assert core.on_status(status(), 1.01) is StatusDisposition.DUPLICATE


def test_bridge_core_sessions_order_limits_and_timeout_rearm():
    core = ready_core()
    decision = core.accept_command(
        vx=4.0,
        wz=-4.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=10.0,
        now_ros_sec=10.01,
        now_monotonic=1.01,
    )
    command = decision.validated_command
    assert decision.disposition is CommandDisposition.ACCEPT_ENABLE
    assert command is not None
    assert command.vx == pytest.approx(0.45) and command.wz == -1.5
    conflicting = core.accept_command(
        vx=0,
        wz=0,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=10.02,
        now_ros_sec=10.03,
        now_monotonic=1.02,
    )
    assert conflicting.disposition is CommandDisposition.GLOBAL_RELEASE
    assert core.snapshot.rearm_required
    rearm = core.accept_command(
        vx=0,
        wz=0,
        enable=True,
        source=3,
        session_id=100,
        sequence=2,
        command_stamp_sec=10.04,
        now_ros_sec=10.05,
        now_monotonic=1.21,
    )
    assert rearm.disposition is CommandDisposition.REQUIRE_REARM


def test_command_and_status_timeouts_release():
    command_core = ready_core()
    command_core.accept_command(
        vx=0.2,
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.02,
    )
    assert command_core.tick(1.20) == (True, "command_timeout")
    assert command_core.snapshot.rearm_required

    status_core = ready_core()
    assert status_core.tick(1.30) == (True, "status_timeout")
    assert status_core.snapshot.rearm_required


def test_duplicate_is_idempotent_and_does_not_refresh_lease():
    core = ready_core()
    accepted = core.accept_command(
        vx=0.2,
        wz=0.1,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=10.0,
        now_ros_sec=10.0,
        now_monotonic=1.02,
    )
    duplicate = core.accept_command(
        vx=0.2,
        wz=0.1,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=999.0,
        now_ros_sec=-999.0,
        now_monotonic=1.10,
    )

    assert accepted.disposition is CommandDisposition.ACCEPT_ENABLE
    assert duplicate.disposition is CommandDisposition.IGNORE_DUPLICATE
    assert core.snapshot.last_command_at == 1.02


def test_stale_new_epoch_is_ignored_but_stale_active_epoch_releases():
    inactive = ready_core()
    before = inactive.snapshot
    stale = inactive.accept_command(
        vx=0.2,
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=10.0,
        now_ros_sec=10.5,
        now_monotonic=1.02,
    )
    assert stale.disposition is CommandDisposition.REJECT_STALE
    assert inactive.snapshot == before

    active = ready_core()
    assert (
        active.accept_command(
            vx=0.2,
            wz=0.0,
            enable=True,
            source=3,
            session_id=100,
            sequence=1,
            command_stamp_sec=10.0,
            now_ros_sec=10.01,
            now_monotonic=1.02,
        ).disposition
        is CommandDisposition.ACCEPT_ENABLE
    )
    stale_active = active.accept_command(
        vx=0.3,
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=2,
        command_stamp_sec=10.0,
        now_ros_sec=10.5,
        now_monotonic=1.03,
    )
    assert stale_active.disposition is CommandDisposition.GLOBAL_RELEASE
    assert active.snapshot.rearm_required
    assert active.snapshot.target_vx == 0.0


def test_old_session_and_out_of_order_do_not_mutate_current_target():
    core = ready_core()
    core.accept_command(
        vx=0.2,
        wz=0.1,
        enable=True,
        source=3,
        session_id=100,
        sequence=10,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.02,
    )
    before = core.snapshot
    stale = core.accept_command(
        vx=0.3,
        wz=0.0,
        enable=True,
        source=3,
        session_id=99,
        sequence=2,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.03,
    )
    out_of_order = core.accept_command(
        vx=0.4,
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=9,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.04,
    )

    assert stale.disposition is CommandDisposition.REJECT_OLD_SESSION
    assert out_of_order.disposition is CommandDisposition.REJECT_OUT_OF_ORDER
    assert core.snapshot == before


def test_active_nonfinite_command_requires_release_and_rearm():
    core = ready_core()
    core.accept_command(
        vx=0.2,
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.02,
    )
    decision = core.accept_command(
        vx=float("nan"),
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=2,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.03,
    )

    assert decision.disposition is CommandDisposition.GLOBAL_RELEASE
    assert core.snapshot.rearm_required
    assert core.snapshot.target_vx == 0.0


def test_bridge_core_disable_enable_edge_recovers_after_fault():
    core = ready_core()
    assert core.on_status(status(sequence=3, flags=1), 1.05)
    core.on_disable_sent(2)
    assert core.snapshot.state is BridgeState.FAULT_LATCHED
    assert core.snapshot.rearm_required
    core.accept_command(
        vx=0,
        wz=0,
        enable=False,
        source=0,
        session_id=101,
        sequence=1,
        command_stamp_sec=2.0,
        now_ros_sec=2.01,
        now_monotonic=1.06,
    )
    assert core.on_status(status(sequence=4), 1.07)
    assert core.snapshot.state is BridgeState.WAIT_FAULT_CLEAR
    core.on_disable_sent(3)
    assert core.on_status(status(sequence=5, received=3, applied=3), 1.08)
    assert core.snapshot.state is BridgeState.WIRE_REARM_READY


class _Serial:
    pass


def test_transport_coalesces_velocity_and_preserves_critical_priority():
    transport = SerialTransportSupervisor(_Serial(), "/dev/null", 115200, outbound_capacity=3)
    assert transport.submit(1, b"old", coalesce=True)
    assert transport.submit(2, b"critical", critical=True)
    assert transport.submit(1, b"new", coalesce=True)
    assert [(item.cmd, item.payload) for item in transport._outbound] == [
        (2, b"critical"),
        (1, b"new"),
    ]


def test_ros_adapter_has_no_external_state_machine_write():
    node_source = (
        Path(__file__).resolve().parents[1] / "stm32_robot_bridge" / "bridge_node_v3.py"
    ).read_text(encoding="utf-8")

    assert "core.snapshot =" not in node_source
    assert "core.machine." not in node_source
