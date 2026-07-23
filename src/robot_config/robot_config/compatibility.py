"""Fail-closed platform compatibility evaluation."""

from dataclasses import dataclass

MISMATCH_COMMIT = 1 << 0
MISMATCH_HARDWARE = 1 << 1
MISMATCH_CAPABILITIES = 1 << 2
MISMATCH_PARAMETER_CRC = 1 << 3
MISMATCH_WHEEL_LAYOUT = 1 << 4


@dataclass(frozen=True)
class CompatibilityResult:
    compatible: bool
    simulated: bool
    mismatch_flags: int
    reason: str


def evaluate_compatibility(
    *,
    simulated,
    firmware_commit,
    expected_firmware_commit,
    hardware_revision,
    expected_hardware_revision,
    capabilities,
    required_capabilities,
    parameter_crc32,
    expected_parameter_crc32,
    enabled_mask,
    expected_enabled_mask,
) -> CompatibilityResult:
    if simulated:
        return CompatibilityResult(True, True, 0, "semantic simulation")
    flags = 0
    if str(firmware_commit).lower() != str(expected_firmware_commit).lower():
        flags |= MISMATCH_COMMIT
    if int(hardware_revision) != int(expected_hardware_revision):
        flags |= MISMATCH_HARDWARE
    if int(capabilities) & int(required_capabilities) != int(required_capabilities):
        flags |= MISMATCH_CAPABILITIES
    if int(expected_parameter_crc32) == 0 or int(parameter_crc32) != int(expected_parameter_crc32):
        flags |= MISMATCH_PARAMETER_CRC
    if int(enabled_mask) & 0x0F != int(expected_enabled_mask) & 0x0F:
        flags |= MISMATCH_WHEEL_LAYOUT
    return CompatibilityResult(
        not flags, False, flags, "compatible" if not flags else "contract mismatch"
    )
