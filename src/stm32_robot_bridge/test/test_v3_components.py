import math

from stm32_robot_bridge.imu_converter import AffineClockSynchronizer, classify_imu_quality
from stm32_robot_bridge.motion_supervisor import MotionSupervisor, SupervisorLevel
from stm32_robot_bridge.odometry import EncoderOdometry, signed_int32_delta


def test_encoder_odometry_integrates_counts_and_wraps():
    odom = EncoderOdometry(
        wheel_radius_m=0.035, track_width_m=0.176, counts_per_revolution=2464, max_dt_sec=1.0
    )
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    result = odom.update((100, 100, 100, 100), sample_time_sec=0.1, hard_max_speed_mps=10.0)
    assert result.integrated and result.pose.x > 0.0
    assert math.isclose(result.pose.yaw, 0.0, abs_tol=1e-12)
    assert signed_int32_delta(-2147483648, 2147483647) == 1


def test_affine_clock_tracks_drift_and_field_quality_is_independent():
    clock = AffineClockSynchronizer(min_samples=5)
    estimate = None
    for index in range(20):
        estimate = clock.update(index * 20, 10.0 + index * 0.02001)
    assert estimate.stable
    assert 0.999 <= estimate.scale <= 1.001
    validity = classify_imu_quality(1 << 2, online=True, error=False)
    assert validity.gyro_valid and not validity.orientation_valid


def test_motion_supervisor_degrades_then_releases():
    supervisor = MotionSupervisor()
    kwargs = dict(
        command_vx=0.2,
        command_wz=0.0,
        wheel_speeds=(0.0, 0.4, 0.0, 0.4),
        wheel_targets=(0.2,) * 4,
        feedback_vx=0.2,
        wheel_wz=0.0,
        gyro_z=0.0,
    )
    first = supervisor.update(now_sec=0.0, **kwargs)
    assert first.level in (SupervisorLevel.DEGRADED, SupervisorLevel.NORMAL)
    final = supervisor.update(now_sec=0.6, **kwargs)
    assert final.level is SupervisorLevel.CRITICAL and final.release_required
