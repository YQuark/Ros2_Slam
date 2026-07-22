from robot_supervision.motion_supervisor import (
    MotionSupervisor,
    SupervisorConfig,
    SupervisorLevel,
    observation_requires_release,
)


def test_supervisor_degrades_and_latches_critical_after_hold():
    supervisor = MotionSupervisor(SupervisorConfig(critical_hold_sec=0.1))
    args = dict(
        command_vx=0.2,
        command_wz=0.0,
        wheel_speeds=(0.0, 0.4, 0.0, 0.4),
        wheel_targets=(0.2,) * 4,
        feedback_vx=0.2,
        wheel_wz=0.0,
        gyro_z=0.0,
    )
    first = supervisor.update(now_sec=1.0, **args)
    critical = supervisor.update(now_sec=1.11, **args)
    assert first.level is not SupervisorLevel.CRITICAL
    assert critical.level is SupervisorLevel.CRITICAL
    assert critical.release_required


def test_missing_gyro_does_not_invalidate_healthy_wheels():
    result = MotionSupervisor().update(
        now_sec=1.0,
        command_vx=0.0,
        command_wz=0.0,
        wheel_speeds=(0.0,) * 4,
        wheel_targets=(0.0,) * 4,
        feedback_vx=0.0,
        wheel_wz=0.0,
        gyro_z=None,
    )
    assert result.level is SupervisorLevel.NORMAL
    assert not result.release_required


def test_schema_or_encoder_anomaly_requires_release():
    assert observation_requires_release(
        schema_version=2, expected_schema_version=1, encoder_anomaly_mask=0
    )
    assert observation_requires_release(
        schema_version=1, expected_schema_version=1, encoder_anomaly_mask=1
    )
    assert not observation_requires_release(
        schema_version=1, expected_schema_version=1, encoder_anomaly_mask=0
    )
