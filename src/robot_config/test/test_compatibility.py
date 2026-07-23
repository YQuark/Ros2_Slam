from robot_config.compatibility import MISMATCH_PARAMETER_CRC, evaluate_compatibility


def test_real_platform_is_fail_closed_while_parameter_crc_is_unknown() -> None:
    result = evaluate_compatibility(
        simulated=False,
        firmware_commit="366a",
        expected_firmware_commit="366a",
        hardware_revision=0x20000,
        expected_hardware_revision=0x20000,
        capabilities=0x1F,
        required_capabilities=0x1F,
        parameter_crc32=123,
        expected_parameter_crc32=0,
        enabled_mask=0b0110,
        expected_enabled_mask=0b0110,
    )
    assert not result.compatible
    assert result.mismatch_flags & MISMATCH_PARAMETER_CRC


def test_simulation_is_explicitly_permitted() -> None:
    result = evaluate_compatibility(
        simulated=True,
        firmware_commit="",
        expected_firmware_commit="x",
        hardware_revision=0,
        expected_hardware_revision=1,
        capabilities=31,
        required_capabilities=31,
        parameter_crc32=0,
        expected_parameter_crc32=1,
        enabled_mask=6,
        expected_enabled_mask=6,
    )
    assert result.compatible and result.simulated
