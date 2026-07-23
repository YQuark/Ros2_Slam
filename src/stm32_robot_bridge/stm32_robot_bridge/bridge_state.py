"""ROS-independent STM32 bridge communication state machine."""

from enum import IntEnum


FAULT_STATUS_MASK = 0x03


class BridgeState(IntEnum):
    DISCONNECTED = 0
    NEGOTIATING = 1
    WAIT_SAFE_STATUS = 2
    WAIT_DISABLE_ACK = 3
    WIRE_SYNCHRONIZED = 4


class BridgeStateMachine:
    def __init__(self) -> None:
        self.state = BridgeState.DISCONNECTED
        self.fault_present = False
        self.expected_disable_sequence: int | None = None

    @property
    def can_drive(self) -> bool:
        return self.state is BridgeState.WIRE_SYNCHRONIZED and not self.fault_present

    def on_serial_opened(self) -> None:
        self.state = BridgeState.NEGOTIATING
        self.fault_present = False
        self.expected_disable_sequence = None

    def on_settled(self) -> None:
        if self.state is BridgeState.NEGOTIATING:
            self.state = BridgeState.WAIT_SAFE_STATUS

    def on_valid_status(self, status_flags: int) -> None:
        has_fault = bool(int(status_flags) & FAULT_STATUS_MASK)
        if has_fault:
            self.fault_present = True
            self.expected_disable_sequence = None
            self.state = BridgeState.WAIT_SAFE_STATUS
            return
        self.fault_present = False

    def on_disable_sent(self, sequence: int) -> None:
        if not self.fault_present and self.state in (
            BridgeState.WAIT_SAFE_STATUS,
            BridgeState.WIRE_SYNCHRONIZED,
        ):
            self.expected_disable_sequence = int(sequence) & 0xFFFFFFFF
            self.state = BridgeState.WAIT_DISABLE_ACK

    def on_disable_applied(self, sequence: int) -> bool:
        if (
            self.state is BridgeState.WAIT_DISABLE_ACK
            and self.expected_disable_sequence == (int(sequence) & 0xFFFFFFFF)
            and not self.fault_present
        ):
            self.state = BridgeState.WIRE_SYNCHRONIZED
            self.expected_disable_sequence = None
            return True
        return False

    def on_drive_enabled(self) -> None:
        return

    def on_disable_command(self) -> None:
        return

    def on_status_timeout(self) -> None:
        if self.state not in (BridgeState.DISCONNECTED, BridgeState.NEGOTIATING):
            self.state = BridgeState.WAIT_SAFE_STATUS
            self.fault_present = False
            self.expected_disable_sequence = None

    def on_disconnected(self) -> None:
        self.state = BridgeState.DISCONNECTED
        self.fault_present = False
        self.expected_disable_sequence = None
