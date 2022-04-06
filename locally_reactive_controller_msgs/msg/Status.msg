# Status options
int8 STOPPED=0      # Robot stopped
int8 RUNNING=1      # Robot moving
int8 UNREACHABLE=2  # Stopped but cannot reach the goal
int8 PAUSED=3       # Not computing anything

# Status
int32 status

# Progress
float32 progress

# 
bool goal_reached

# Header
Header header