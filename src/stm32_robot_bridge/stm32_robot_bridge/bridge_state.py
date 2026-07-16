"""ROS-independent STM32 bridge communication state machine."""

from enum import IntEnum


FAULT_STATUS_MASK = 0x03


class BridgeState(IntEnum):
    DISCONNECTED = 0
    SETTLING = 1
    WAIT_STATUS = 2
    READY = 3
    ACTIVE = 4
    FAULT = 5


class BridgeStateMachine:
    def __init__(self) -> None:
        self.state = BridgeState.DISCONNECTED
        self.fault_present = False
        self.recovery_disable_seen = False

    @property
    def can_drive(self) -> bool:
        return (
            self.state in (BridgeState.READY, BridgeState.ACTIVE)
            and not self.fault_present
        )

    def on_serial_opened(self) -> None:
        self.state = BridgeState.SETTLING
        self.fault_present = False
        self.recovery_disable_seen = False

    def on_settled(self) -> None:
        if self.state is BridgeState.SETTLING:
            self.state = BridgeState.WAIT_STATUS

    def on_valid_status(self, status_flags: int) -> None:
        has_fault = bool(int(status_flags) & FAULT_STATUS_MASK)
        if has_fault:
            if self.state is not BridgeState.FAULT:
                self.recovery_disable_seen = False
            self.fault_present = True
            self.state = BridgeState.FAULT
            return

        self.fault_present = False
        if self.state is BridgeState.FAULT:
            if self.recovery_disable_seen:
                self.state = BridgeState.READY
            return
        if self.state is BridgeState.WAIT_STATUS:
            self.state = BridgeState.READY

    def on_drive_enabled(self) -> None:
        if self.can_drive:
            self.state = BridgeState.ACTIVE

    def on_disable_command(self) -> None:
        if self.state is BridgeState.FAULT:
            self.recovery_disable_seen = True
            if not self.fault_present:
                self.state = BridgeState.READY
            return
        if self.state is BridgeState.ACTIVE:
            self.state = BridgeState.READY

    def on_status_timeout(self) -> None:
        if self.state not in (BridgeState.DISCONNECTED, BridgeState.SETTLING):
            self.state = BridgeState.WAIT_STATUS
            self.fault_present = False

    def on_disconnected(self) -> None:
        self.state = BridgeState.DISCONNECTED
        self.fault_present = False
        self.recovery_disable_seen = False
