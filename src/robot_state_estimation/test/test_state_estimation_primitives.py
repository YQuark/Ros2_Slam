import math
import pytest

from robot_state_estimation.imu_processing import classify_quality
from robot_state_estimation.time_mapper import (
    McuClockMapper,
    SampleDisposition,
    SampleOrderTracker,
)
from robot_state_estimation.wheel_odometry import (
    EncoderOdometry,
    covariance_multiplier,
    signed_int32_delta,
)


def make_odom():
    return EncoderOdometry(
        wheel_radius_m=0.05,
        track_width_m=0.20,
        counts_per_revolution=1000,
        max_dt_sec=0.25,
    )


def test_encoder_odometry_validates_geometry_and_count_shape():
    with pytest.raises(ValueError):
        EncoderOdometry(wheel_radius_m=0.0, track_width_m=0.2, counts_per_revolution=1000)
    odom = make_odom()
    with pytest.raises(ValueError):
        odom.update((0, 0, 0), sample_time_sec=0.0)


def test_order_tracker_separates_sessions_duplicates_and_old_samples():
    tracker = SampleOrderTracker()
    assert tracker.update(1, 10) is SampleDisposition.FIRST
    assert tracker.update(1, 10) is SampleDisposition.DUPLICATE
    assert tracker.update(1, 9) is SampleDisposition.OUT_OF_ORDER
    assert tracker.update(1, 11) is SampleDisposition.FORWARD
    assert tracker.update(2, 0) is SampleDisposition.FIRST


def test_sensor_reset_allows_sequence_restart_in_same_transport_session():
    tracker = SampleOrderTracker()
    assert tracker.update(7, 100) is SampleDisposition.FIRST
    assert tracker.update(7, 0) is SampleDisposition.OUT_OF_ORDER
    tracker.reset()
    assert tracker.update(7, 0) is SampleDisposition.FIRST


def test_clock_mapper_uses_sensor_delta_and_handles_wrap():
    mapper = McuClockMapper(min_samples=2)
    first = mapper.update(0xFFFFFFF0, 10.0)
    second = mapper.update(0x00000004, 10.02)
    assert second.sensor_time_sec - first.sensor_time_sec == pytest.approx(0.020)
    assert second.sample_ros_sec > first.sample_ros_sec


def test_encoder_odometry_integrates_straight_and_rejects_anomaly():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    update = odom.update((100, 100, 100, 100), sample_time_sec=0.1)
    assert update.integrated and update.pose.x > 0.0
    before = update.pose
    rejected = odom.update((200, 200, 200, 200), sample_time_sec=0.2, anomaly_mask=1)
    assert not rejected.integrated and rejected.pose == before


def test_encoder_odometry_integrates_rotation_and_arc():
    rotation = make_odom()
    rotation.update((0, 0, 0, 0), sample_time_sec=0.0)
    turned = rotation.update((-100, -100, 100, 100), sample_time_sec=0.1)
    assert turned.integrated
    assert turned.pose.yaw > 0.0
    assert abs(turned.pose.x) < 1e-9

    arc = make_odom()
    arc.update((0, 0, 0, 0), sample_time_sec=0.0)
    curved = arc.update((50, 50, 100, 100), sample_time_sec=0.1)
    assert curved.integrated
    assert curved.pose.x > 0.0 and curved.pose.y > 0.0 and curved.pose.yaw > 0.0


def test_default_m2_m3_layout_does_not_halve_distance():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0, enabled_mask=0b0110, speed_valid_mask=0b0110)
    update = odom.update(
        (0, 100, 100, 0),
        sample_time_sec=0.1,
        enabled_mask=0b0110,
        speed_valid_mask=0b0110,
    )
    expected = 100 * odom.meters_per_count
    assert update.integrated
    assert update.left_distance == pytest.approx(expected)
    assert update.right_distance == pytest.approx(expected)


def test_layout_change_resets_integration_baseline():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    changed = odom.update(
        (0, 100, 100, 0),
        sample_time_sec=0.1,
        enabled_mask=0b0110,
        speed_valid_mask=0b0110,
    )
    assert not changed.integrated

    odom.reset_sample_baseline()
    missing_side = odom.update(
        (0, 0, 0, 0),
        sample_time_sec=0.2,
        enabled_mask=0b0010,
        speed_valid_mask=0b0010,
    )
    assert not missing_side.integrated


def test_reset_generation_resets_baseline_even_with_same_wire_session():
    odom = make_odom()
    odom.update(
        (0, 0, 0, 0), sample_time_sec=0.0, transport_session_id=7, reset_generation=1
    )
    reset = odom.update(
        (100, 100, 100, 100),
        sample_time_sec=0.1,
        transport_session_id=7,
        reset_generation=2,
    )
    assert not reset.integrated


@pytest.mark.parametrize(
    "enabled,counts",
    [
        (0b0110, (0, 100, 100, 0)),
        (0b1111, (100, 100, 100, 100)),
        (0b1110, (0, 100, 100, 100)),
        (0b0111, (100, 100, 100, 0)),
    ],
)
def test_two_and_four_drive_layouts_share_one_distance_algorithm(enabled, counts):
    odom = make_odom()
    odom.update(
        (0, 0, 0, 0), sample_time_sec=0.0, enabled_mask=enabled, speed_valid_mask=enabled
    )
    update = odom.update(
        counts, sample_time_sec=0.1, enabled_mask=enabled, speed_valid_mask=enabled
    )
    assert update.integrated
    assert update.pose.x == pytest.approx(100 * odom.meters_per_count)


def test_implausible_enabled_wheel_jump_is_rejected():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    jump = odom.update((100000, 0, 0, 0), sample_time_sec=0.1)
    assert not jump.integrated


def test_time_regression_and_long_gap_do_not_move_pose():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=1.0)
    regressed = odom.update((10, 10, 10, 10), sample_time_sec=0.9)
    assert not regressed.integrated and regressed.pose.x == 0.0
    long_gap = odom.update((20, 20, 20, 20), sample_time_sec=2.0)
    assert not long_gap.integrated and long_gap.pose.x == 0.0


def test_covariance_multiplier_increases_for_age_turn_disagreement_and_quality():
    nominal = covariance_multiplier(
        wheel_speeds=(0.1, 0.1, 0.1, 0.1),
        speed_valid_mask=0x0F,
        sample_age_sec=0.0,
        turn_rate=0.0,
    )
    degraded = covariance_multiplier(
        wheel_speeds=(0.1, 0.3, 0.1, -0.1),
        speed_valid_mask=0x07,
        sample_age_sec=0.1,
        turn_rate=0.5,
        quality_flags=3,
    )
    assert nominal == 1.0
    assert degraded > nominal

    two_wheel = covariance_multiplier(
        wheel_speeds=(0.0, 0.1, 0.1, 0.0),
        enabled_mask=0b0110,
        speed_valid_mask=0b0110,
        sample_age_sec=0.0,
        turn_rate=0.0,
    )
    assert two_wheel == 3.0


def test_signed_int32_delta_wraps():
    assert signed_int32_delta(-2147483648, 2147483647) == 1
    assert signed_int32_delta(2147483647, -2147483648) == -1


def test_twist_covariance_is_per_sample_not_accumulated_pose_uncertainty():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    updates = []
    for index in range(1, 51):
        updates.append(
            odom.update(
                (index * 10,) * 4,
                sample_time_sec=index * 0.1,
                hard_max_speed_mps=2.0,
            )
        )
    assert updates[-1].pose_covariance[0] > updates[0].pose_covariance[0]
    assert updates[-1].twist_covariance == pytest.approx(updates[0].twist_covariance)


def test_imu_field_quality_degrades_orientation_independently():
    validity = classify_quality(1 << 2, status_flags=1)
    assert validity.gyro_valid
    assert validity.accel_valid
    assert not validity.orientation_valid
