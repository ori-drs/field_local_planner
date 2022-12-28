# State options
int8 NOT_READY = 0  # Initial state
int8 FINISHED = 1   # Robot at goal. Do nothing
int8 EXECUTING = 2  # Robot trying to reach the goal
int8 FAILURE = 5    # Robot cannot make any progress

# Status
int32 state

# Progress
float32 progress

# 
bool goal_reached

# Header
Header header