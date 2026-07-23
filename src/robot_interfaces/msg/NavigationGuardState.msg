uint8 STATE_CLOSED=0
uint8 STATE_CANCEL_PENDING=1
uint8 STATE_WAIT_OLD_GOAL_TERMINATED=2
uint8 STATE_READY_FOR_NEW_GOAL=3
uint8 STATE_ACTIVE=4

uint32 REASON_NONE=0
uint32 REASON_LINK=1
uint32 REASON_FIRMWARE=2
uint32 REASON_SUPERVISION=4
uint32 REASON_COMPATIBILITY=8
uint32 REASON_HOST_CONTROL=16

std_msgs/Header header
uint8 state
uint32 revoke_generation
uint32 goal_generation
uint32 cancel_attempts
uint32 reason_flags
string active_goal_uuid
string revoked_goal_uuid
