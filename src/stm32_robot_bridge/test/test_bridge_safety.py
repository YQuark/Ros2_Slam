from stm32_robot_bridge.bridge_state import BridgeState, BridgeStateMachine


def test_state_machine_requires_status_before_drive_and_releases_on_timeout():
    machine = BridgeStateMachine()
    assert machine.state is BridgeState.DISCONNECTED
    assert machine.can_drive is False

    machine.on_serial_opened()
    assert machine.state is BridgeState.NEGOTIATING
    machine.on_settled()
    assert machine.state is BridgeState.WAIT_SAFE_STATUS
    machine.on_valid_status(status_flags=0)
    assert machine.state is BridgeState.WAIT_SAFE_STATUS
    assert machine.can_drive is False

    machine.on_disable_sent(7)
    assert machine.state is BridgeState.WAIT_DISABLE_ACK
    assert machine.on_disable_applied(6) is False
    assert machine.on_disable_applied(7) is True
    assert machine.state is BridgeState.WIRE_SYNCHRONIZED
    assert machine.can_drive is True

    machine.on_drive_enabled()
    assert machine.state is BridgeState.WIRE_SYNCHRONIZED
    machine.on_status_timeout()
    assert machine.state is BridgeState.WAIT_SAFE_STATUS
    assert machine.can_drive is False


def test_fault_recovery_requires_disable_then_enable_edge():
    machine = BridgeStateMachine()
    machine.on_serial_opened()
    machine.on_settled()
    machine.on_valid_status(status_flags=0)
    machine.on_disable_sent(1)
    machine.on_disable_applied(1)
    machine.on_drive_enabled()

    machine.on_valid_status(status_flags=0x02)
    assert machine.state is BridgeState.WAIT_SAFE_STATUS
    machine.on_valid_status(status_flags=0)
    assert machine.state is BridgeState.WAIT_SAFE_STATUS
    assert machine.can_drive is False

    machine.on_disable_sent(2)
    assert machine.state is BridgeState.WAIT_DISABLE_ACK
    assert machine.on_disable_applied(2)
    assert machine.state is BridgeState.WIRE_SYNCHRONIZED
    assert machine.can_drive is True
    machine.on_drive_enabled()
    assert machine.state is BridgeState.WIRE_SYNCHRONIZED


def test_disconnect_always_returns_to_disconnected():
    machine = BridgeStateMachine()
    machine.on_serial_opened()
    machine.on_settled()
    machine.on_valid_status(status_flags=0)

    machine.on_disconnected()

    assert machine.state is BridgeState.DISCONNECTED
    assert machine.can_drive is False
