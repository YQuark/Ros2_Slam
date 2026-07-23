uint8 HOST_SUBSOURCE_NONE=0
uint8 HOST_SUBSOURCE_TELEOP=1
uint8 HOST_SUBSOURCE_TEST=2
uint8 HOST_SUBSOURCE_NAV=3
uint8 HOST_SUBSOURCE_RESEARCH=4

std_msgs/Header header
geometry_msgs/Twist twist
bool enable
uint8 host_subsource
uint64 command_epoch
uint32 sequence
