Header header

### constants:

# LED colors:
int8 COLOR_UNKNOWN=-1
int8 COLOR_RED=0
int8 COLOR_YELLOW=1
int8 COLOR_GREEN=2
int8 COLOR_BLUE=3

# individual LED behaviors:
int8 BEHAVIOR_UNKNOWN=-1
int8 BEHAVIOR_OFF=0
int8 BEHAVIOR_ON=1
int8 BEHAVIOR_TOGGLE=2
int8 BEHAVIOR_PULSE=3

# group LED behaviors:
int8 GROUP_UNKNOWN=-1
int8 GROUP_NO_LED=0
int8 GROUP_SINGLE_LED=1
int8 GROUP_ALL_LEDS=2
int8 GROUP_CASCADE_LEDS_FORWARD=3
int8 GROUP_CASCADE_LEDS_REVERSE=4

int8 behavior
int8 group_behavior

# use LED color values in this array.
int8[] active_leds
int8   num_leds
