uint8 SCHEMA_VERSION=1

# header.stamp is the host ROS receive time. The estimator maps
# mcu_sample_time_ms into ROS time. header.frame_id is the configured base frame.
std_msgs/Header header
uint8 schema_version
uint64 transport_session_id
uint32 reset_generation
uint32 status_sequence
uint32 mcu_sample_time_ms

int32[4] encoder_count
float32[4] wheel_speed_mps
float32[4] wheel_target_mps
float32[4] motor_current_a
int16[4] motor_output_permille

uint8 motor_enabled_mask
uint8 speed_valid_mask
uint8 encoder_anomaly_mask
uint8 side_consistency_flags
uint8 comm_health_flags
uint8 status_flags
uint32 error_flags
uint32 latched_error_flags
