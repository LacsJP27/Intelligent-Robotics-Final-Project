# --- Safety ---
EMERGENCY_STOP_DISTANCE = 0.05   # meters — stop if anything closer than this
FORWARD_ARC_HALF_DEG = 90.0      # degrees — check this cone left/right of forward

# --- Tour State Machine ---
WAYPOINT_DWELL_TIME = 2.0        # seconds — pause at each waypoint before advancing
TELEOP_TIMEOUT_SEC = 0.5         # seconds — resume Nav2 this long after last key press
FSM_LOOP_RATE_HZ = 10.0  

# --- Group Tracker ---
GROUP_LOST_TIMEOUT_SEC = 4        # seconds — if group not detected for this long, consider lost
GROUP_THRESHOLD_DISTANCE = 0.3048     # meters — if group is farther than this, consider lost

# --- Topic Names ---
TOPIC_SCAN = '/scan'
TOPIC_CMD_VEL = '/cmd_vel'
TOPIC_TELEOP = '/cmd_vel_key'
TOPIC_TOUR_STATE = '/tour/state'
TOPIC_EMERGENCY = '/emergency_stop'
TOPIC_GROUP_DETECTED = '/group/detected'
