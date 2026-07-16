import pytest

from stm32_robot_bridge.bridge_core import BridgeCore
from stm32_robot_bridge.bridge_state import BridgeState
from stm32_robot_bridge.motion_supervisor import MotionSupervisor, SupervisorConfig, SupervisorLevel
from stm32_robot_bridge.protocol_v3 import (
    ACK_APPLIED,
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
        ACK_APPLIED,
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
    core.on_startup_released()
    assert core.on_status(status(), 1.0)
    assert core.snapshot.state is BridgeState.READY
    return core


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
    core.on_startup_released()
    assert core.on_status(status(), 1.0)
    assert not core.on_status(status(), 1.01)


def test_bridge_core_sessions_order_limits_and_timeout_rearm():
    core = ready_core()
    command = core.accept_command(
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
    assert command.vx == 0.45 and command.wz == -1.5
    with pytest.raises(ValueError, match="duplicate"):
        core.accept_command(
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
    assert core.tick(1.20) == (True, "command_timeout")
    with pytest.raises(ValueError, match="rearm"):
        core.accept_command(
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


def test_bridge_core_disable_enable_edge_recovers_after_fault():
    core = ready_core()
    assert core.on_status(status(sequence=2, flags=1), 1.05)
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
    assert core.on_status(status(sequence=3), 1.07)
    assert core.snapshot.state is BridgeState.READY


def test_motion_supervisor_disabled_and_hysteretic_clear():
    disabled = MotionSupervisor(SupervisorConfig(enabled=False))
    result = disabled.update(
        now_sec=0,
        command_vx=0,
        command_wz=0,
        wheel_speeds=(10,) * 4,
        wheel_targets=(0,) * 4,
        feedback_vx=10,
        wheel_wz=0,
        gyro_z=0,
    )
    assert result.level is SupervisorLevel.NORMAL and result.command_scale == 1.0


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
