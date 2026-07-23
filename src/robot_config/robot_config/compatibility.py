"""Fail-closed platform compatibility evaluation."""

from dataclasses import dataclass

MISMATCH_COMMIT = 1 << 0
MISMATCH_HARDWARE = 1 << 1
MISMATCH_CAPABILITIES = 1 << 2
MISMATCH_PARAMETER_CRC = 1 << 3
MISMATCH_WHEEL_LAYOUT = 1 << 4
MISMATCH_PROTOCOL = 1 << 5
MISMATCH_SCHEMA = 1 << 6


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
    protocol_version=3,
    expected_protocol_version=3,
    schema_version=1,
    expected_schema_version=1,
) -> CompatibilityResult:
    flags = 0
    # Semantic simulation may skip physical identity only.  It still exercises
    # the public protocol, capability and wheel-layout contracts.
    if not simulated and str(firmware_commit).lower() != str(expected_firmware_commit).lower():
        flags |= MISMATCH_COMMIT
    if not simulated and int(hardware_revision) != int(expected_hardware_revision):
        flags |= MISMATCH_HARDWARE
    if int(protocol_version) != int(expected_protocol_version):
        flags |= MISMATCH_PROTOCOL
    if int(schema_version) != int(expected_schema_version):
        flags |= MISMATCH_SCHEMA
    if int(capabilities) & int(required_capabilities) != int(required_capabilities):
        flags |= MISMATCH_CAPABILITIES
    if not simulated and (
        int(expected_parameter_crc32) == 0 or int(parameter_crc32) != int(expected_parameter_crc32)
    ):
        flags |= MISMATCH_PARAMETER_CRC
    if int(enabled_mask) & 0x0F != int(expected_enabled_mask) & 0x0F:
        flags |= MISMATCH_WHEEL_LAYOUT
    return CompatibilityResult(
        not flags,
        bool(simulated),
        flags,
        (
            ("semantic simulation" if simulated and not flags else "compatible")
            if not flags
            else "contract mismatch"
        ),
    )
