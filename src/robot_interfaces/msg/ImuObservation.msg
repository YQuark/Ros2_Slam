uint8 SCHEMA_VERSION=1

# Raw Upper-v3 IMU observation. Unit conversion and quality interpretation are
# owned by robot_state_estimation.
std_msgs/Header header
uint8 schema_version
uint64 transport_session_id
uint32 mcu_sample_time_ms
uint32 sensor_time
uint32 sample_sequence

float32[3] acceleration_g
float32[3] angular_velocity_dps
float32[3] euler_deg
geometry_msgs/Quaternion orientation
uint32 quality_flags
uint32[7] quality_counters
uint8 status_flags
int8 temperature_c
