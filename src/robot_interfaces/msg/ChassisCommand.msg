uint8 SOURCE_NONE=0
uint8 SOURCE_TELEOP=1
uint8 SOURCE_TEST=2
uint8 SOURCE_NAV=3
uint8 SOURCE_RESEARCH=4

std_msgs/Header header
geometry_msgs/Twist twist
bool enable
uint8 source
uint64 session_id
uint32 sequence
