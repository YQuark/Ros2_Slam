import pytest

from robot_chassis_model.wheel_layout import WheelLayout, WheelLayoutError


def test_default_m2_m3_aggregates_without_halving() -> None:
    layout = WheelLayout(0b0110, 0b0110)
    assert layout.aggregate((0.0, 1.0, 1.0, 0.0)) == (1.0, 1.0)
    assert layout.pair_disagreement((0.0, 1.0, 1.0, 0.0)) == 0.0


def test_disabled_wheels_do_not_count_as_invalid() -> None:
    layout = WheelLayout(0b0110, 0b1111)
    assert layout.enabled_invalid_count == 0


def test_missing_side_is_rejected() -> None:
    with pytest.raises(WheelLayoutError):
        WheelLayout(0b0010, 0b0010).aggregate((0.0, 1.0, 0.0, 0.0))
