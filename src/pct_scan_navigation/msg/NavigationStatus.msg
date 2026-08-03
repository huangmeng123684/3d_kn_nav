std_msgs/Header header

uint8 IDLE=0
uint8 WAITING_GOAL=1
uint8 PLANNING_GLOBAL=2
uint8 GLOBAL_READY=3
uint8 PLANNING_LOCAL=4
uint8 NAVIGATING=5
uint8 AVOIDING=6
uint8 BLOCKED=7
uint8 GOAL_REACHED=8
uint8 CANCELED=9
uint8 FAILED=10
uint8 LOCALIZATION_LOST=11
uint8 MAP_SWITCHING=12
uint8 state

string map_name
bool goal_active
float32 distance_to_goal
uint32 remaining_waypoints
string reason
