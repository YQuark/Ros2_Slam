uint8 STATE_CLOSED=0
uint8 STATE_READY_FOR_NEW_GOAL=1
uint8 STATE_ACTIVE=2

std_msgs/Header header
uint8 state
uint32 revoke_generation
uint32 reason_flags
string active_goal_uuid
string revoked_goal_uuid
