uint8 LEVEL_UNKNOWN=0
uint8 LEVEL_NORMAL=1
uint8 LEVEL_WARN=2
uint8 LEVEL_DEGRADED=3
uint8 LEVEL_CRITICAL=4

uint32 REASON_NONE=0
uint32 REASON_WHEEL_PAIR=1
uint32 REASON_TRACKING=2
uint32 REASON_YAW_DISAGREEMENT=4
uint32 REASON_UNEXPECTED_MOTION=8
uint32 REASON_OBSERVATION_STALE=16
uint32 REASON_IMU_UNAVAILABLE=32
uint32 REASON_OBSERVATION_INVALID=64

std_msgs/Header header
uint8 level
uint32 reason_flags
float32 score
float32 command_scale
bool release_required
uint64 command_session_id
uint32 command_sequence
uint64 observation_session_id
uint32 observation_sequence
uint32 observation_age_ms
string config_sha256
