# --- Group Detection ---
REAR_ARC_MIN_DEG = 120.0        # degrees (0 = forward)
REAR_ARC_MAX_DEG = 240.0
DETECTION_RANGE_MIN = 0.5       # meters (ignore noise close to robot)
DETECTION_RANGE_MAX = 5.0       # meters
DBSCAN_EPS = 0.3                # meters — cluster neighborhood radius
DBSCAN_MIN_SAMPLES = 3          # min points per cluster
CLUSTER_WIDTH_MIN = 0.1         # meters — reject noise
CLUSTER_WIDTH_MAX = 0.8         # meters — reject walls
STATIC_THRESHOLD = 0.05         # meters — movement below this = static
STATIC_FRAME_COUNT = 5          # consecutive static frames to classify as static

# --- Tour State Machine ---
GROUP_TOO_FAR_THRESHOLD = 3.0   # meters — triggers PAUSED_WAITING
GROUP_RESUME_THRESHOLD = 2.0    # meters — resumes from PAUSED (hysteresis)
GROUP_LOST_TIMEOUT = 5.0        # seconds — triggers SEARCHING
SEARCH_TIMEOUT = 30.0           # seconds — abort tour if can't find group
SEARCH_ANGULAR_VEL = 0.3        # rad/s — rotation speed during search
WAYPOINT_DWELL_TIME = 2.0       # seconds — pause at each waypoint

# --- Safety ---
EMERGENCY_STOP_DISTANCE = 0.25  # meters — immediate stop
FORWARD_ARC_HALF_DEG = 90.0     # degrees — half-width of forward safety arc
SAFETY_PUBLISH_RATE = 20.0      # Hz

# --- Topic Names ---
TOPIC_SCAN = '/scan'
TOPIC_GROUP_STATUS = '/group_tracker/status'
TOPIC_TOUR_STATE = '/tour/state'
TOPIC_SAFETY_CMD = '/safety/cmd_vel'
TOPIC_NAV_CMD = '/nav_vel'
TOPIC_CMD_VEL = '/cmd_vel'
TOPIC_TELEOP = '/cmd_vel_key'
TOPIC_HAZARD = '/hazard_detection'
