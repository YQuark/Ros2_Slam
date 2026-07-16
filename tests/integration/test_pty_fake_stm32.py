import math
import os
import select
import struct

import pytest
import serial

from stm32_robot_bridge.bridge_state import BridgeState, BridgeStateMachine
from stm32_robot_bridge.command_guard import CommandGuard, CommandRejected
from stm32_robot_bridge.protocol_v2 import (
    CMD_SET_VELOCITY,
    CMD_STATUS,
    CommandStream,
    FrameParser,
    build_frame,
    decode_status_payload,
)
from stm32_robot_bridge.serial_transport import TransportStats, write_all


def status_payload(status_flags=0, *, version=2):
    payload = bytearray(65)
    payload[0] = version
    payload[1] = status_flags
    payload[2] = 0
    payload[3] = 0b0110
    payload[62] = 0b0110
    return bytes(payload)


class FakeSTM32:
    def __init__(self, master_fd):
        self.master_fd = master_fd
        self.parser = FrameParser()

    def send_status(self, status_flags=0, *, version=2):
        os.write(
            self.master_fd, build_frame(CMD_STATUS, status_payload(status_flags, version=version))
        )

    def receive_frames(self):
        ready, _, _ = select.select([self.master_fd], [], [], 0.5)
        assert ready, "timed out waiting for upper serial frame"
        return self.parser.feed(os.read(self.master_fd, 4096))


def host_receive_status(device, parser, state_machine):
    ready, _, _ = select.select([device.fileno()], [], [], 0.5)
    assert ready, "timed out waiting for fake STM32 STATUS"
    frames = parser.feed(device.read(device.in_waiting or 1))
    assert len(frames) == 1 and frames[0][0] == CMD_STATUS
    status = decode_status_payload(frames[0][1])
    assert status is not None
    state_machine.on_valid_status(status.status_flags)
    return status


def decode_velocity_frame(frames):
    assert len(frames) == 1 and frames[0][0] == CMD_SET_VELOCITY
    vx, wz, enable, mode = struct.unpack("<ffBB", frames[0][1])
    assert math.isfinite(vx) and math.isfinite(wz)
    return vx, wz, enable, mode


@pytest.mark.serial
def test_pty_fake_stm32_exercises_drive_release_fault_and_reconnect():
    master_fd, slave_fd = os.openpty()
    slave_name = os.ttyname(slave_fd)
    os.close(slave_fd)
    device = serial.Serial(slave_name, 115200, timeout=0.0, write_timeout=0.2)
    try:
        fake = FakeSTM32(master_fd)
        host_parser = FrameParser()
        stats = TransportStats()
        state = BridgeStateMachine()
        guard = CommandGuard(
            hard_max_linear_mps=0.45,
            hard_max_angular_radps=1.50,
            max_command_age_sec=0.15,
            status_timeout_sec=0.25,
        )
        stream = CommandStream(cmd_timeout_sec=0.15, keepalive_sec=0.05)

        state.on_serial_opened()
        state.on_settled()
        assert state.state is BridgeState.WAIT_STATUS
        with pytest.raises(CommandRejected, match="STATUS unavailable"):
            guard.validate(
                vx=0.2,
                wz=0.1,
                command_stamp_sec=1.0,
                now_sec=1.0,
                status_age_sec=None,
                drive_permitted=state.can_drive,
                status_flags=0,
                protocol_version=2,
            )

        fake.send_status()
        status = host_receive_status(device, host_parser, state)
        command = guard.validate(
            vx=0.8,
            wz=-2.0,
            command_stamp_sec=1.0,
            now_sec=1.0,
            status_age_sec=0.0,
            drive_permitted=state.can_drive,
            status_flags=status.status_flags,
            protocol_version=status.version,
        )
        assert command.vx == 0.45 and command.wz == -1.50
        stream.update_command(command.vx, command.wz, 1.0)
        state.on_drive_enabled()
        sender = lambda payload: write_all(device, build_frame(CMD_SET_VELOCITY, payload), stats)

        assert stream.tick(1.0, sender) is True
        assert decode_velocity_frame(fake.receive_frames())[:3] == (pytest.approx(0.45), -1.5, 1)
        assert stream.tick(1.051, sender) is True
        assert decode_velocity_frame(fake.receive_frames())[2] == 1
        assert stream.tick(1.151, sender) is True
        assert decode_velocity_frame(fake.receive_frames())[:3] == (0.0, 0.0, 0)
        assert stream.tick(1.30, sender) is False

        stream.update_command(0.1, 0.0, 2.0)
        assert stream.tick(2.0, sender) is True
        assert decode_velocity_frame(fake.receive_frames())[2] == 1
        with pytest.raises(CommandRejected, match="not finite"):
            guard.validate(
                vx=float("nan"),
                wz=0.0,
                command_stamp_sec=2.0,
                now_sec=2.0,
                status_age_sec=0.0,
                drive_permitted=state.can_drive,
                status_flags=0,
                protocol_version=2,
            )
        assert stream.release(sender, 2.0) is True
        assert decode_velocity_frame(fake.receive_frames())[:3] == (0.0, 0.0, 0)

        state.on_status_timeout()
        assert state.state is BridgeState.WAIT_STATUS and not state.can_drive
        state.on_disconnected()
        state.on_serial_opened()
        state.on_settled()
        assert state.state is BridgeState.WAIT_STATUS
        fake.send_status(status_flags=1)
        host_receive_status(device, host_parser, state)
        assert state.state is BridgeState.FAULT and not state.can_drive
    finally:
        device.close()
        os.close(master_fd)
