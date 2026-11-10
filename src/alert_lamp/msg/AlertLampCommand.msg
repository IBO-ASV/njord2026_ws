std_msgs/Header header

uint8 color
uint8 pattern

float32 period
float32 duty_ratio

string reason

uint8 COLOR_OFF=0
uint8 COLOR_GREEN=1
uint8 COLOR_YELLOW=2
uint8 COLOR_RED=4

# COLOR_* values are bit flags and may be combined, for example
# COLOR_GREEN | COLOR_RED.

uint8 PATTERN_OFF=0
uint8 PATTERN_SOLID=1
uint8 PATTERN_BLINK=2
