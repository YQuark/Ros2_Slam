from robot_verification.fake_base_model import (
    FakeBaseModel,
    StatusSampleLatch,
    signed_int32,
)


def test_fake_model_has_symmetric_straight_counts():
    model = FakeBaseModel(
        wheel_radius_m=0.05,
        track_width_m=0.20,
        counts_per_revolution=1000,
        response_tau_sec=0.01,
    )
    model.set_target(0.2, 0.0)
    sample = model.step(0.1)
    assert sample.encoder_counts[0] == sample.encoder_counts[1]
    assert sample.encoder_counts[2] == sample.encoder_counts[3]
    assert sample.wheel_speeds[0] == sample.wheel_speeds[2]


def test_fake_model_rotation_has_opposite_sides_and_int32_wrap():
    model = FakeBaseModel(
        wheel_radius_m=0.05,
        track_width_m=0.20,
        counts_per_revolution=1000,
        response_tau_sec=0.01,
    )
    model.set_target(0.0, 1.0)
    sample = model.step(0.1)
    assert sample.wheel_speeds[0] < 0.0 < sample.wheel_speeds[2]
    assert signed_int32(1 << 31) == -(1 << 31)


def test_duplicate_status_reuses_sequence_and_complete_sample_snapshot():
    latch = StatusSampleLatch()
    first_sequence, first = latch.update(("sample-a", 20), repeat=False)
    repeated_sequence, repeated = latch.update(("sample-b", 40), repeat=True)
    next_sequence, next_sample = latch.update(("sample-c", 60), repeat=False)

    assert repeated_sequence == first_sequence
    assert repeated == first
    assert next_sequence == first_sequence + 1
    assert next_sample == ("sample-c", 60)
