import struct

from robot_verification.firmware_emulator import FirmwareV3Emulator


def velocity(session, sequence, enable=True, vx=0.2, wz=0.0):
    return struct.pack("<BffBBQI", 3, vx, wz, int(enable), 2, session, sequence)


def test_duplicate_keepalive_refreshes_watchdog_without_changing_sequence():
    firmware = FirmwareV3Emulator()
    assert firmware.apply_velocity(velocity(7, 1, False), 0.0)
    assert firmware.apply_velocity(velocity(7, 2, True), 0.01)
    for step in range(1, 101):
        assert firmware.apply_velocity(velocity(7, 2, True), step * 0.05)
        firmware.tick(step * 0.05)
        assert firmware.host_enabled


def test_watchdog_fault_rearm_reboot_and_retired_session_are_fail_closed():
    firmware = FirmwareV3Emulator()
    assert firmware.apply_velocity(velocity(7, 1, False), 0.0)
    assert firmware.apply_velocity(velocity(7, 2, True), 0.01)
    firmware.tick(0.22)
    assert not firmware.host_enabled
    firmware.set_fault()
    assert not firmware.apply_velocity(velocity(7, 3, True), 0.23)
    assert firmware.clear_fault()
    firmware.reboot()
    assert not firmware.apply_velocity(velocity(7, 4, False), 0.24)
    assert firmware.apply_velocity(velocity(8, 1, False), 0.25)
    assert firmware.apply_velocity(velocity(8, 2, True), 0.26)


def test_conflicting_duplicate_and_out_of_order_are_rejected_without_target_change():
    firmware = FirmwareV3Emulator()
    assert firmware.apply_velocity(velocity(7, 1, False), 0.0)
    assert firmware.apply_velocity(velocity(7, 2, True, vx=0.2), 0.01)

    assert not firmware.apply_velocity(velocity(7, 2, True, vx=0.3), 0.02)
    assert firmware.host_enabled
    assert not firmware.apply_velocity(velocity(7, 1, True, vx=0.2), 0.03)
    assert firmware.host_enabled


def test_sequence_wraparound_is_forward_and_layout_masks_are_explicit():
    firmware = FirmwareV3Emulator(enabled_mask=0x06)
    assert firmware.apply_velocity(velocity(7, 0xFFFFFFFE, False), 0.0)
    assert firmware.apply_velocity(velocity(7, 0xFFFFFFFF, True), 0.01)
    assert firmware.apply_velocity(velocity(7, 0, True), 0.02)

    payload = firmware.status_payload()
    assert payload[3] == 0x06
    assert payload[62] == 0x06
    assert payload[63] == 0
