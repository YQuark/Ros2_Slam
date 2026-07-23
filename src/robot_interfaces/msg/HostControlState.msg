uint8 STATE_DISABLED=0
uint8 STATE_WAIT_SUPERVISION=1
uint8 STATE_READY=2
uint8 STATE_ACTIVE=3
uint8 STATE_WAIT_SOURCE_QUIET=4
uint8 STATE_WAIT_FRESH_SOURCE=5

uint8 REJECT_NONE=0
uint8 REJECT_INVALID=1
uint8 REJECT_STALE=2
uint8 REJECT_REARM_REQUIRED=3
uint8 REJECT_SUPERVISION_STALE=4
uint8 REJECT_MOTION_CRITICAL=5

uint32 REARM_NONE=0
uint32 REARM_TRANSPORT=1
uint32 REARM_STATUS_TIMEOUT=2
uint32 REARM_ACK_TIMEOUT=4
uint32 REARM_FIRMWARE_REJECT=8
uint32 REARM_ESTOP=16
uint32 REARM_FAULT=32
uint32 REARM_COMMAND_TIMEOUT=64
uint32 REARM_MOTION_CRITICAL=128
uint32 REARM_SUPERVISION_STALE=256

std_msgs/Header header
uint8 control_state
uint8 selected_host_subsource
uint32 command_age_ms
uint8 command_reject_reason
bool enable_intent
bool rearm_required
uint32 rearm_reason_flags
uint64 command_epoch
uint32 command_sequence
string config_sha256
