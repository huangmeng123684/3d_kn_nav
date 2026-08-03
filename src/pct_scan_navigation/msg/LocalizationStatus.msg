std_msgs/Header header

uint8 UNINITIALIZED=0
uint8 INITIALIZING=1
uint8 INIT_SUCCESS=2
uint8 TRACKING=3
uint8 TRACKING_WARN=4
uint8 TRACKING_LOST=5
uint8 MAP_SWITCHING=6
uint8 state

string map_name
float32 fitness
string reason
