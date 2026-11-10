uint8 STATE_IDLE=0
uint8 STATE_VALIDATING=1
uint8 STATE_CONFIGURING=2
uint8 STATE_WAITING_FOR_READINESS=3
uint8 STATE_WAITING_FOR_AUTO_PERMISSION=4
uint8 STATE_RUNNING=5
uint8 STATE_PAUSED=6
uint8 STATE_CANCELING=7
uint8 STATE_SUCCEEDED=8
uint8 STATE_FAILED=9
uint8 STATE_REJECTED=10

uint8 MODE_MANUAL=0
uint8 MODE_AUTO=1

uint8 SOURCE_ZERO=0
uint8 SOURCE_MANUAL=1
uint8 SOURCE_AUTO=2
uint8 SOURCE_SBUS=3

builtin_interfaces/Time stamp
uint8 state
string task_id
string execution_id
string stage
float32 progress
string message
uint8 requested_control_mode
uint8 effective_control_source
bool auto_permitted
string[] inhibit_reasons
builtin_interfaces/Time last_transition_time
string required_nav2_profile
string active_nav2_profile
uint8 runtime_state
