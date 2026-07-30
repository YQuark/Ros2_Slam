"""ROS-independent admission policy for formal odometry evidence."""


def evidence_matches_odometry(
    input_stamp_ns: int, evidence_stamp_ns: int, max_skew_sec: float
) -> bool:
    """Accept estimator latency without allowing unrelated stale/future odometry."""

    max_skew_ns = max(int(float(max_skew_sec) * 1_000_000_000), 0)
    return abs(int(input_stamp_ns) - int(evidence_stamp_ns)) <= max_skew_ns
