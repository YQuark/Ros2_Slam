uint8 STATE_DISCONNECTED=0
uint8 STATE_SETTLING=1
uint8 STATE_WAIT_STATUS=2
uint8 STATE_READY=3
uint8 STATE_ACTIVE=4
uint8 STATE_FAULT=5

std_msgs/Header header
uint8 bridge_state
uint8 selected_source
uint32 command_sequence
bool upper_enabled
uint8 firmware_control_source
uint8 status_flags
uint32 error_flags
uint32 latched_error_flags
