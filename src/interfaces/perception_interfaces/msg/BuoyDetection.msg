uint8 CLASS_GREEN=0
uint8 CLASS_RED=1
uint8 CLASS_NORTH=2
uint8 CLASS_EAST=3
uint8 CLASS_SOUTH=4
uint8 CLASS_WEST=5

uint8 POSITION_NONE=0
uint8 POSITION_ZED_DEPTH=1
uint8 POSITION_LIDAR_FUSED=2

uint8 class_id
float32 confidence
geometry_msgs/Point position
uint8 position_source
