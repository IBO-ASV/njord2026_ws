uint8 SET_MODE=1
uint8 TASK_CHECK=2
uint8 TASK_START=3
uint8 TASK_STOP=4
uint8 TASK_STATUS=5
uint8 UM982_HOT_RESTART=6
uint8 RESTART_BRINGUP=7

uint8 MINIPC=1
uint8 JETSON=2

uint64 request_id
uint8 command
uint8 target
uint8 requested_mode
string task_id
