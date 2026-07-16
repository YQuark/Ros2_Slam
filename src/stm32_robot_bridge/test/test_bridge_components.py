import math
from unittest.mock import MagicMock

import pytest

from stm32_robot_bridge.diagnostics import WarningThrottle
from stm32_robot_bridge.imu_converter import (
    GRAVITY_MPS2,
    ImuClockSynchronizer,
    InvalidImuSample,
    accel_g_to_mps2,
    gyro_dps_to_rad,
    normalize_quaternion,
)
from stm32_robot_bridge.odometry import (
    DynamicCovarianceModel,
    DifferentialOdometry,
    Pose2D,
    apply_deadzone,
    integrate_midpoint,
    wrap_angle,
)
from stm32_robot_bridge.serial_transport import (
    SerialShortWrite,
    TransportStats,
    open_serial_port,
    reset_serial_buffers,
    set_modem_lines_low,
    write_all,
)


def test_midpoint_odometry_and_angle_wrap_are_pure():
    pose = integrate_midpoint(Pose2D(1.0, 2.0, 3.0), vx=0.5, wz=1.0, dt=0.4)

    assert math.isclose(pose.x, 1.0 + 0.2 * math.cos(3.2))
    assert math.isclose(pose.y, 2.0 + 0.2 * math.sin(3.2))
    assert math.isclose(pose.yaw, wrap_angle(3.4))
    assert apply_deadzone(0.009, 0.01) == 0.0
    assert apply_deadzone(0.01, 0.01) == 0.01


def test_imu_unit_conversion_and_quaternion_validation():
    assert normalize_quaternion((0.0, 0.0, 0.0, 2.0)) == (0.0, 0.0, 0.0, 1.0)
    assert normalize_quaternion((float("nan"), 0.0, 0.0, 1.0)) is None
    assert normalize_quaternion((0.0, 0.0, 0.0, 0.0)) is None
    assert gyro_dps_to_rad((180.0, -90.0, 0.0)) == (math.pi, -math.pi / 2.0, 0.0)
    accel = accel_g_to_mps2((1.0, -1.0, 0.5))
    assert accel == (GRAVITY_MPS2, -GRAVITY_MPS2, GRAVITY_MPS2 / 2.0)


def test_serial_transport_configures_port_without_ros_dependency():
    serial_module = MagicMock()
    device = serial_module.Serial.return_value

    opened = open_serial_port(serial_module, "/dev/ttyTEST", 230400)

    assert opened is device
    assert device.port == "/dev/ttyTEST"
    assert device.baudrate == 230400
    assert device.timeout == 0.0
    assert device.write_timeout == 0.2
    assert device.rtscts is False
    assert device.dsrdtr is False
    device.open.assert_called_once_with()


def test_serial_line_and_buffer_helpers_tolerate_optional_failures():
    device = MagicMock()
    type(device).dtr = MagicMock(side_effect=OSError("unsupported"))

    set_modem_lines_low(device)
    reset_serial_buffers(device)

    device.reset_input_buffer.assert_called_once_with()
    device.reset_output_buffer.assert_called_once_with()


def test_warning_throttle_keeps_rate_limit_state_outside_ros_node():
    throttle = WarningThrottle()

    assert throttle.should_emit("crc", now=10.0, interval_sec=2.0) is True
    assert throttle.should_emit("crc", now=11.9, interval_sec=2.0) is False
    assert throttle.should_emit("crc", now=12.0, interval_sec=2.0) is True


def test_write_all_completes_segmented_short_writes():
    device = MagicMock()
    device.write.side_effect = [3, 4]

    written = write_all(device, b"1234567")

    assert written == 7
    assert device.write.call_count == 2
    assert device.write.call_args_list[0].args[0] == b"1234567"
    assert device.write.call_args_list[1].args[0] == b"4567"


def test_write_all_rejects_zero_or_invalid_progress():
    device = MagicMock()
    device.write.return_value = 0
    with pytest.raises(SerialShortWrite):
        write_all(device, b"frame")

    device.write.return_value = None
    with pytest.raises(SerialShortWrite):
        write_all(device, b"frame")


def test_transport_stats_start_at_zero():
    stats = TransportStats()
    assert stats.tx_frames == 0
    assert stats.tx_short_writes == 0
    assert stats.rx_frames == 0
    assert stats.serial_reconnects == 0


def test_differential_odometry_integrates_each_new_sample_once():
    odom = DifferentialOdometry(max_dt_sec=0.25)

    first = odom.update_from_wheel_speeds(0.5, 0.0, sample_time_sec=1.0, trusted=True)
    second = odom.update_from_wheel_speeds(0.5, 0.0, sample_time_sec=1.1, trusted=True)
    duplicate = odom.update_from_wheel_speeds(0.5, 0.0, sample_time_sec=1.1, trusted=True)

    assert first.integrated is False
    assert math.isclose(second.pose.x, 0.05)
    assert second.integrated is True
    assert duplicate.integrated is False
    assert duplicate.pose == second.pose


def test_differential_odometry_does_not_fill_large_status_gap():
    odom = DifferentialOdometry(max_dt_sec=0.25)
    odom.update_from_wheel_speeds(0.5, 0.0, sample_time_sec=1.0, trusted=True)

    gap = odom.update_from_wheel_speeds(0.5, 0.0, sample_time_sec=2.0, trusted=True)
    resumed = odom.update_from_wheel_speeds(0.5, 0.0, sample_time_sec=2.1, trusted=True)

    assert gap.integrated is False
    assert gap.trusted is False
    assert gap.pose == Pose2D()
    assert math.isclose(resumed.pose.x, 0.05)


def test_differential_odometry_reset_clears_pose_and_sample_baseline():
    odom = DifferentialOdometry(max_dt_sec=0.25)
    odom.update_from_wheel_speeds(0.5, 0.2, sample_time_sec=1.0, trusted=True)
    odom.update_from_wheel_speeds(0.5, 0.2, sample_time_sec=1.1, trusted=True)

    odom.reset()

    assert odom.pose == Pose2D()
    assert odom.last_sample_time_sec is None


def test_imu_clock_uses_mcu_time_with_filtered_receive_offset():
    clock = ImuClockSynchronizer(offset_alpha=0.1)

    first = clock.update(timestamp_ms=1000, sample_count=1, receive_ros_sec=10.0)
    second = clock.update(timestamp_ms=1020, sample_count=2, receive_ros_sec=10.04)

    assert first.sample_ros_sec == 10.0
    assert second.sample_ros_sec == pytest.approx(10.022)
    assert second.sample_ros_sec > first.sample_ros_sec
    assert second.dropped_samples == 0


def test_imu_clock_counts_drops_and_rejects_duplicate_samples():
    clock = ImuClockSynchronizer()
    clock.update(timestamp_ms=1000, sample_count=1, receive_ros_sec=10.0)

    update = clock.update(timestamp_ms=1060, sample_count=4, receive_ros_sec=10.06)

    assert update.dropped_samples == 2
    assert clock.total_dropped_samples == 2
    with pytest.raises(InvalidImuSample, match="duplicate"):
        clock.update(timestamp_ms=1080, sample_count=4, receive_ros_sec=10.08)


def test_imu_clock_handles_uint32_wrap_and_detects_mcu_reset():
    clock = ImuClockSynchronizer()
    before_wrap = clock.update(
        timestamp_ms=0xFFFFFFF0,
        sample_count=0xFFFFFFFE,
        receive_ros_sec=100.0,
    )
    after_wrap = clock.update(timestamp_ms=20, sample_count=1, receive_ros_sec=100.036)

    assert after_wrap.sample_ros_sec > before_wrap.sample_ros_sec
    assert after_wrap.reset is False

    reset = clock.update(timestamp_ms=5, sample_count=0, receive_ros_sec=101.0)
    assert reset.reset is True
    assert reset.sample_ros_sec == 101.0


def test_dynamic_odom_covariance_grows_with_motion_dt_and_wheel_disagreement():
    model = DynamicCovarianceModel()

    calm = model.estimate(vx=0.05, wz=0.0, dt=0.02, wheel_disagreement=0.0, trusted=True)
    stressed = model.estimate(vx=0.4, wz=1.0, dt=0.2, wheel_disagreement=0.3, trusted=True)

    assert stressed.linear_variance > calm.linear_variance
    assert stressed.angular_variance > calm.angular_variance
    assert stressed.pose_xy_variance > calm.pose_xy_variance


def test_dynamic_odom_covariance_marks_untrusted_sample_explicitly():
    model = DynamicCovarianceModel()

    covariance = model.estimate(
        vx=0.0,
        wz=0.0,
        dt=0.02,
        wheel_disagreement=0.0,
        trusted=False,
    )

    assert covariance.trusted is False
    assert covariance.linear_variance == 1000.0
    assert covariance.angular_variance == 1000.0
