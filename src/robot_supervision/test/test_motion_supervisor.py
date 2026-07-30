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
    assert critical.release_host_candidate


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
    assert not result.release_host_candidate


def test_schema_or_encoder_anomaly_requires_release():
    assert observation_requires_release(
        schema_version=2, expected_schema_version=1, encoder_anomaly_mask=0
    )
    assert not observation_requires_release(
        schema_version=1, expected_schema_version=1, encoder_anomaly_mask=1
    )
    assert observation_requires_release(
        schema_version=1,
        expected_schema_version=1,
        encoder_anomaly_mask=0,
        enabled_mask=0b0110,
        speed_valid_mask=0b0010,
    )
    assert not observation_requires_release(
        schema_version=1, expected_schema_version=1, encoder_anomaly_mask=0
    )


def test_default_two_wheel_layout_has_no_pair_disagreement() -> None:
    result = MotionSupervisor().update(
        now_sec=1.0,
        command_vx=0.2,
        command_wz=0.0,
        wheel_speeds=(0.0, 0.2, 0.2, 0.0),
        wheel_targets=(0.0, 0.2, 0.2, 0.0),
        enabled_mask=0b0110,
        speed_valid_mask=0b0110,
        feedback_vx=0.2,
        wheel_wz=0.0,
        gyro_z=None,
    )
    assert result.reason == "wheel_pair"
    assert result.score == 0.0


def test_wheel_only_unexpected_rotation_is_critical_after_hold() -> None:
    supervisor = MotionSupervisor(SupervisorConfig(critical_hold_sec=0.0))
    result = supervisor.update(
        now_sec=1.0,
        command_vx=0.0,
        command_wz=0.0,
        wheel_speeds=(0.0, -0.2, 0.2, 0.0),
        wheel_targets=(0.0,) * 4,
        enabled_mask=0b0110,
        speed_valid_mask=0b0110,
        feedback_vx=0.0,
        wheel_wz=2.0,
        gyro_z=None,
    )
    assert result.level is SupervisorLevel.CRITICAL
    assert result.reason == "unexpected_motion"


def test_stale_command_excludes_tracking_and_unexpected_motion_components() -> None:
    result = MotionSupervisor(SupervisorConfig(critical_hold_sec=0.0)).update(
        now_sec=1.0,
        command_vx=0.0,
        command_wz=0.0,
        wheel_speeds=(0.2,) * 4,
        wheel_targets=(1.0,) * 4,
        feedback_vx=0.2,
        wheel_wz=0.0,
        gyro_z=None,
        command_valid=False,
    )
    components = dict(result.components)

    assert components["tracking"] == 0.0
    assert components["unexpected_motion"] == 0.0
    assert not result.release_host_candidate


def test_supervision_score_is_exposed_as_components_not_probability() -> None:
    result = MotionSupervisor().update(
        now_sec=1.0,
        command_vx=0.2,
        command_wz=0.0,
        wheel_speeds=(0.0, 0.2, 0.2, 0.0),
        wheel_targets=(0.0, 0.2, 0.2, 0.0),
        enabled_mask=0b0110,
        speed_valid_mask=0b0110,
        feedback_vx=0.2,
        wheel_wz=0.0,
        gyro_z=None,
    )

    assert set(dict(result.components)) == {
        "wheel_pair",
        "tracking",
        "yaw",
        "unexpected_motion",
    }
