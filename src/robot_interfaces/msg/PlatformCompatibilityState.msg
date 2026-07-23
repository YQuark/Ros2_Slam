uint8 STATE_UNKNOWN=0
uint8 STATE_COMPATIBLE=1
uint8 STATE_SIMULATION=2
uint8 STATE_INCOMPATIBLE=3

std_msgs/Header header
uint8 state
bool permit_formal_odometry
bool permit_navigation
uint32 mismatch_flags
uint64 wire_session_id
uint64 observation_session_id
bool observation_ready
string reason
