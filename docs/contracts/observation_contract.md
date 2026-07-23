# Observation contract

Bridge publishes Wheel/IMU observations only for new sample identities. Duplicate and out-of-order STATUS affect diagnostics but never sample freshness. Consumers independently reject non-forward `(wire_session,status_sequence)` and IMU `(wire_session,sample_count,sensor_time)` identities. ROS receive time is metadata; MCU sample time drives integration after clock mapping.
