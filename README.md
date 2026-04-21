# SEC Robot Tour Guide — Final Project Plan

> **Course**: CS 4023 — Intelligent Mobile Robotics (University of Oklahoma)
> **Demo Week**: May 4–8, 2026 | **Draft Report Due**: April 23
> **Team Size**: 3 | **Weight**: 50% of course grade
> **Platform**: TurtleBot 4 (Create 3 + RPi 4) · ROS 2 Jazzy · Gazebo Harmonic

---

## Mission Statement

Build a robot tour guide that leads people through waypoints inside SEC (Sarkeys Energy Center). The robot navigates ahead of a group, detects whether people are following via rear-arc LiDAR clustering, and adapts its behavior (pause, search, resume) accordingly.

---

## Minimum Viable Demo (MVP)

The MVP is the baseline that **must work** for demo day:

1. Robot navigates 3+ waypoints sequentially via Nav2
2. Robot pauses if no LiDAR clusters are detected behind it
3. Robot resumes navigation when clusters reappear
4. Emergency stop triggers when an obstacle is within 0.25m
5. Human override via keyboard teleop at any time

Everything beyond this (camera fusion, narration, dynamic speed modulation) is a stretch goal.

---

## Folder Structure

```
sec_tour_guide_ws/
├── src/
│   └── sec_tour_guide/                    # Main ROS 2 Python package
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── sec_tour_guide             # ament resource index marker
│       │
│       ├── sec_tour_guide/                # Python module
│       │   ├── __init__.py
│       │   ├── tour_state_machine.py      # FSM: tour orchestration node
│       │   ├── group_tracker.py           # LiDAR rear-arc clustering node
│       │   ├── safety_monitor.py          # Emergency stop + bump sensor node
│       │   └── utils/
│       │       ├── __init__.py
│       │       ├── clustering.py          # DBSCAN / Euclidean clustering helpers
│       │       ├── geometry.py            # Polar→Cartesian, yaw helpers, quaternion utils
│       │       └── constants.py           # Shared thresholds, topic names, frame IDs
│       │
│       ├── msg/
│       │   └── GroupStatus.msg            # Custom message for group detection
│       │
│       ├── config/
│       │   ├── nav2_params.yaml           # Nav2 parameter overrides for TurtleBot 4
│       │   ├── twist_mux.yaml             # twist_mux priority config
│       │   └── tour_waypoints.yaml        # Waypoint sequence (x, y, yaw per stop)
│       │
│       ├── launch/
│       │   ├── tour_guide.launch.py       # Full system launch (all nodes + Nav2 + mux)
│       │   ├── sim_tour.launch.py         # Gazebo sim launch (world + robot + people)
│       │   └── detection_test.launch.py   # Standalone group_tracker + rviz for tuning
│       │
│       ├── worlds/
│       │   ├── sec_corridor.sdf           # Simplified SEC hallway world for Gazebo Harmonic
│       │   └── models/
│       │       └── person_cylinder/
│       │           ├── model.config
│       │           └── model.sdf          # Cylinder person model (r=0.15m, h=1.7m)
│       │
│       ├── maps/
│       │   ├── sec_corridor.pgm           # Occupancy grid image
│       │   └── sec_corridor.yaml          # Map metadata (resolution, origin)
│       │
│       ├── rviz/
│       │   └── tour_guide.rviz           # Pre-configured RViz layout
│       │
│       └── test/
│           ├── test_clustering.py         # Unit tests for clustering logic
│           ├── test_state_machine.py      # Unit tests for FSM transitions
│           └── test_group_tracker.py      # Integration test with recorded LaserScan
│
├── docs/
│   ├── architecture.md                    # Node graph + topic diagram
│   ├── attribution.md                     # What's ours vs. borrowed (academic integrity)
│   └── testing_results.md                 # Metrics, screenshots, rosbag analysis
│
├── bags/                                  # Recorded rosbag2 data (gitignored, local only)
│   └── .gitkeep
│
├── .gitignore
└── README.md                              # Build instructions, quick start, team info
```

---

## Files — Purpose & Contents

### Core Nodes

| File | Description | Publishes | Subscribes |
|------|-------------|-----------|------------|
| `tour_state_machine.py` | FSM orchestrator. Sends Nav2 goals, listens to group status, manages tour state transitions. | `/tour/state` (`String`) | `/group_tracker/status` (`GroupStatus`), Nav2 action feedback |
| `group_tracker.py` | Filters `/scan` to rear arc, runs DBSCAN clustering, tracks clusters across frames, publishes group status. | `/group_tracker/status` (`GroupStatus`), `/group_tracker/markers` (`MarkerArray` for RViz) | `/scan` (`LaserScan`) |
| `safety_monitor.py` | Monitors `/scan` for emergency-close obstacles (<0.25m). Publishes zero-velocity override via `twist_mux`. Also subscribes to Create 3 hazard detection. | `/safety/cmd_vel` (`Twist`) | `/scan` (`LaserScan`), `/hazard_detection` (Create 3) |

### Custom Message

**`GroupStatus.msg`**
```
std_msgs/Header header
bool group_detected
int32 person_count
float32 avg_distance        # meters from robot center
float32 closest_distance    # meters, nearest cluster
float32 group_bearing       # radians in base_link frame
```

### Config Files

**`tour_waypoints.yaml`** — edit this to define the tour route:
```yaml
tour:
  frame_id: "map"
  waypoints:
    - {x: 5.0,  y: 0.0, yaw: 0.0,   label: "Hallway Start"}
    - {x: 10.0, y: 3.0, yaw: 1.57,   label: "Corner Turn"}
    - {x: 10.0, y: 8.0, yaw: 1.57,   label: "Lab Entrance"}
    - {x: 15.0, y: 8.0, yaw: 0.0,    label: "Tour End"}
```

**`twist_mux.yaml`** — velocity priority:
```yaml
twist_mux:
  ros__parameters:
    topics:
      safety:
        topic: /safety/cmd_vel
        timeout: 0.5
        priority: 0          # highest — emergency stop
      teleop:
        topic: /cmd_vel_key
        timeout: 0.5
        priority: 1          # human override
      navigation:
        topic: /nav_vel
        timeout: 0.5
        priority: 2          # Nav2 output
    output_topic: /cmd_vel
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      DELIBERATIVE LAYER                      │
│                                                              │
│   ┌──────────────────────┐     ┌──────────────────────────┐ │
│   │  tour_state_machine  │────▶│  Nav2 (NavigateToPose)   │ │
│   │  (FSM orchestrator)  │◀────│  action server           │ │
│   └──────────┬───────────┘     └────────────┬─────────────┘ │
│              │                               │               │
│              │ /group_tracker/status          │ /nav_vel      │
│              │                               │               │
├──────────────┼───────────────────────────────┼───────────────┤
│              │      REACTIVE LAYER           │               │
│              │                               │               │
│   ┌──────────┴───────────┐     ┌─────────────▼─────────────┐│
│   │   group_tracker      │     │       twist_mux           ││
│   │   (LiDAR clustering) │     │  P0: safety  ──▶ /cmd_vel ││
│   └──────────▲───────────┘     │  P1: teleop               ││
│              │                 │  P2: nav                   ││
│   ┌──────────┴───────────┐     └─────────────▲─────────────┘│
│   │   safety_monitor     │                   │               │
│   │   (emergency stop)   │───────────────────┘               │
│   └──────────▲───────────┘   /safety/cmd_vel                 │
│              │                                               │
├──────────────┼───────────────────────────────────────────────┤
│              │         HARDWARE / SENSORS                     │
│              │                                               │
│         /scan (RPLIDAR A1)    /odom    /hazard_detection     │
│              │                  │            │                │
│         ┌────┴──────────────────┴────────────┴──────────┐    │
│         │         TurtleBot 4 (Create 3 + RPi 4)        │    │
│         └───────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

---

## Tour State Machine

```
IDLE
  │  [operator starts tour via service call or topic]
  ▼
WAITING_FOR_GROUP
  │  [group_detected == true && closest_distance < 3.0m]
  ▼
NAVIGATING_TO_WAYPOINT
  ├─ [avg_distance > 3.0m]         ──▶ PAUSED_WAITING
  ├─ [group_detected == false > 5s] ──▶ SEARCHING_FOR_GROUP
  ├─ [waypoint reached]            ──▶ WAYPOINT_REACHED
  └─ [emergency obstacle]          ──▶ EMERGENCY_STOP

PAUSED_WAITING
  ├─ [closest_distance < 2.0m]     ──▶ NAVIGATING_TO_WAYPOINT  (hysteresis)
  └─ [group_detected == false > 5s] ──▶ SEARCHING_FOR_GROUP

SEARCHING_FOR_GROUP
  │  [slow 360° in-place rotation]
  ├─ [group re-detected]           ──▶ NAVIGATING_TO_WAYPOINT
  └─ [timeout 30s]                 ──▶ IDLE  (abort tour)

WAYPOINT_REACHED
  ├─ [more waypoints remaining]    ──▶ WAITING_FOR_GROUP  (2s dwell)
  └─ [last waypoint]               ──▶ TOUR_COMPLETE

EMERGENCY_STOP
  └─ [obstacle cleared]            ──▶ NAVIGATING_TO_WAYPOINT

TOUR_COMPLETE
  └─ [reset]                       ──▶ IDLE
```

**Hysteresis note**: Pause triggers at >3.0m, resume triggers at <2.0m. This 1m dead zone prevents oscillation.

---

## Key Parameters & Constants

Define these in `utils/constants.py` for easy tuning:

```python
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
SAFETY_PUBLISH_RATE = 20.0      # Hz

# --- Topic Names ---
TOPIC_SCAN = "/scan"
TOPIC_GROUP_STATUS = "/group_tracker/status"
TOPIC_TOUR_STATE = "/tour/state"
TOPIC_SAFETY_CMD = "/safety/cmd_vel"
TOPIC_NAV_CMD = "/nav_vel"
TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_TELEOP = "/cmd_vel_key"
```

---

## Phased Implementation Plan

### Phase 1 — Foundation (Days 1–7): "Robot navigates waypoints"

**Goal**: TurtleBot 4 autonomously navigates a sequence of waypoints in Gazebo using Nav2.

| Task | Owner | Details |
|------|-------|---------|
| Workspace scaffolding | All | Create `sec_tour_guide_ws/`, `package.xml`, `setup.py`, folder structure. Push to Git. |
| Map creation | **Member A** | Build simplified SEC corridor map. Option 1: `slam_toolbox` in Gazebo. Option 2: hand-draw occupancy grid in GIMP (faster). Output: `sec_corridor.pgm` + `.yaml`. |
| Nav2 configuration | **Member A** | Copy TurtleBot 4 Nav2 defaults, override in `nav2_params.yaml`. Tune costmap inflation, controller velocity limits. Verify `NavigateToPose` action works with the map. |
| `tour_state_machine` skeleton | **Member B** | Implement FSM with states `IDLE → NAVIGATING_TO_WAYPOINT → WAYPOINT_REACHED → TOUR_COMPLETE`. Load waypoints from YAML. Send goals to Nav2 one at a time. No group detection yet — just auto-advance. |
| `twist_mux` setup | **Member C** | Install and configure `twist_mux`. Verify priority works: teleop overrides Nav2. Write `twist_mux.yaml`. |
| `safety_monitor` node | **Member C** | Subscribe to `/scan`, check for any point < 0.25m in forward arc. Publish zero `Twist` to `/safety/cmd_vel`. Subscribe to `/hazard_detection` for bump sensors. |
| `sim_tour.launch.py` | **Member C** | Launch file that brings up Gazebo world + robot + Nav2 + twist_mux + all custom nodes. |

**Phase 1 exit criteria**:
- [ ] Robot spawns in Gazebo corridor world
- [ ] Robot navigates to 3+ waypoints in sequence without collision
- [ ] Teleop overrides Nav2 via `twist_mux`
- [ ] Emergency stop triggers when object placed <0.25m in front

---

### Phase 2 — Group Detection (Days 5–14, overlaps Phase 1)

**Goal**: `group_tracker` node reliably detects simulated people behind the robot.

| Task | Owner | Details |
|------|-------|---------|
| `GroupStatus.msg` | **Member B** | Define custom message, update `package.xml` and `CMakeLists.txt` (or `setup.py` with `rosidl`). |
| `clustering.py` utilities | **Member B** | Implement: polar-to-cartesian conversion, DBSCAN wrapper, cluster size filtering, static-vs-dynamic classification (frame history buffer). |
| `group_tracker` node | **Member B** | Subscribe `/scan` → filter rear arc → cluster → track → publish `GroupStatus`. Also publish `MarkerArray` to `/group_tracker/markers` for RViz visualization. |
| Person cylinder model | **Member C** | Create SDF model: cylinder r=0.15m, h=1.7m, non-static. Place in `worlds/models/person_cylinder/`. |
| Gazebo people spawning | **Member C** | Script or launch file that spawns 2–3 person cylinders behind the robot. Create a simple Python script to move them via Gazebo's `set_entity_pose` service for repeatable testing. |
| `detection_test.launch.py` | **Member C** | Standalone launch: Gazebo + robot (stationary) + cylinders + `group_tracker` + RViz. For tuning detection without Nav2. |
| Unit tests | **Member B** | `test_clustering.py`: feed synthetic LaserScan data, verify cluster output. `test_group_tracker.py`: record a rosbag in Gazebo, replay and verify `GroupStatus` output. |
| Nav2 tuning continued | **Member A** | Fine-tune recovery behaviors, costmap parameters. Test with narrow corridors and obstacles. Prepare for integration. |

**Phase 2 exit criteria**:
- [ ] `group_tracker` correctly reports `group_detected=true` when cylinders are 1–4m behind robot
- [ ] `group_tracker` reports `group_detected=false` when no cylinders present
- [ ] `person_count` approximately matches actual cylinder count
- [ ] `avg_distance` is within ±0.3m of ground truth
- [ ] Static objects (walls) are filtered out
- [ ] RViz markers show detected clusters in correct positions

---

### Phase 3 — Integration: Adaptive Leading (Days 12–21)

**Goal**: Full tour with pause/resume/search behavior driven by group detection.

| Task | Owner | Details |
|------|-------|---------|
| FSM integration | **Member A** | Wire `group_tracker/status` into `tour_state_machine`. Implement all state transitions: `WAITING_FOR_GROUP`, `PAUSED_WAITING`, `SEARCHING_FOR_GROUP`. |
| Nav2 pause/resume | **Member A** | On pause: `cancel_goal_async()`. On resume: re-send current waypoint goal. Handle Nav2 feedback for waypoint-reached detection. |
| Search rotation | **Member A** | In `SEARCHING_FOR_GROUP`: publish slow angular velocity (0.3 rad/s) to Nav2's cmd topic via twist_mux. Stop when `group_detected` fires or timeout. |
| End-to-end Gazebo test | **Member B** | Create test scenario: cylinders follow robot → cylinders stop (triggers pause) → cylinders resume (robot resumes) → cylinders disappear (triggers search). |
| Threshold tuning | **Member B** | Run detection in multiple scenarios, adjust DBSCAN params and distance thresholds. Document final values. |
| `tour_guide.launch.py` | **Member C** | Full system launch: Gazebo + Nav2 + all custom nodes + twist_mux + RViz. Single command to start everything. |
| FSM unit tests | **Member C** | `test_state_machine.py`: Mock `GroupStatus` messages, verify correct state transitions without hardware. |

**Phase 3 exit criteria**:
- [ ] Robot waits at start until group is detected behind it
- [ ] Robot navigates to waypoint, pauses when group falls behind >3m
- [ ] Robot resumes when group catches up to <2m
- [ ] Robot enters search rotation when group is lost >5s
- [ ] Robot aborts tour if group not found within 30s search
- [ ] Full tour completes with no crashes or deadlocks

---

### Phase 4 — Polish, Real-World & Report (Days 19–28)

**Goal**: Demo-ready system, recorded data, written report.

| Task | Owner | Details |
|------|-------|---------|
| Real-world testing | **Member A** | Deploy to physical TurtleBot 4 in SEC. Tune thresholds for real human legs (noisier than cylinders). Record rosbag data. |
| Code cleanup | **All** | Docstrings on every class/method. Type hints. Consistent naming. Remove dead code. Ensure `pylint`/`flake8` passes. |
| `attribution.md` | **Member B** | Document all borrowed code: Nav2 (Apache 2.0), twist_mux, TurtleBot 4 packages, sklearn DBSCAN. Clearly separate original work. |
| Report — Architecture & Methods | **Member B** | System diagram, node descriptions, FSM design, detection algorithm explanation. |
| Report — Results & Testing | **Member C** | Run 5+ tours in simulation, record metrics. Create tables/graphs: completion rate, pause events, timing. Screenshots from RViz and Gazebo. |
| Report — Introduction & Conclusion | **Member A** | Problem statement, related work, lessons learned. |
| Demo video | **Member C** | Screen-record Gazebo demo. Optionally record real-world demo in SEC. |
| README.md | **Member A** | Build instructions, dependencies, quick-start guide, team member contributions. |

**Phase 4 exit criteria**:
- [ ] 5+ successful tour completions recorded (simulation)
- [ ] At least 1 real-world run attempted (even if imperfect)
- [ ] Report draft complete with all sections
- [ ] Code passes linting, all files documented
- [ ] Demo video recorded
- [ ] `attribution.md` complete

---

## Team Member Assignment Summary

| Area | Member A | Member B | Member C |
|------|----------|----------|----------|
| **Phase 1** | Map + Nav2 config | FSM skeleton + waypoints | twist_mux + safety_monitor + launch |
| **Phase 2** | Nav2 tuning continued | group_tracker + clustering + custom msg | Gazebo people sim + cylinder models |
| **Phase 3** | FSM integration + Nav2 pause/resume | E2E testing + threshold tuning | Full launch file + FSM unit tests |
| **Phase 4** | Real-world testing + README + report intro | Attribution + report architecture | Report results + demo video |

**Parallel workstreams**: Phases 1 and 2 overlap (days 5–7). Member B can start `group_tracker` while A and C finish Nav2/mux setup. Phase 3 can begin as soon as Phase 1 core is working (Nav2 navigates waypoints) even if Phase 2 detection isn't perfect yet — use a mock `GroupStatus` publisher for FSM development.

---

## Nav2 Integration Notes

**Action client pattern** (in `tour_state_machine.py`):
```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# Send goal
self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
goal = NavigateToPose.Goal()
goal.pose = self.waypoints[self.current_waypoint_index]
self.goal_handle = await self.nav_client.send_goal_async(goal)

# Pause: cancel current goal
await self.goal_handle.cancel_goal_async()

# Resume: re-send same waypoint
self.goal_handle = await self.nav_client.send_goal_async(goal)
```

**Do NOT** run the existing obstacle avoidance node simultaneously with Nav2's local planner. Nav2 handles obstacle avoidance via its costmap. The `safety_monitor` only fires for emergency stop and publishes through `twist_mux` at highest priority.

---

## Group Detection Algorithm

```
1. Receive LaserScan message
2. Filter to rear arc (120°–240° in base_link frame)
3. Filter by range (0.5m – 5.0m)
4. Convert remaining points: polar → Cartesian (x, y)
5. DBSCAN clustering (eps=0.3m, min_samples=3)
6. For each cluster:
   a. Compute bounding width → reject if < 0.1m or > 0.8m
   b. Compare centroid to previous frame → classify static (Δ < 0.05m for 5 frames) or dynamic
   c. Reject static clusters (walls, furniture)
7. Count remaining dynamic clusters → person_count
8. Compute avg_distance and closest_distance from remaining clusters
9. Publish GroupStatus message
```

**Dependencies**: `scikit-learn` for DBSCAN (pip install), or hand-roll simple Euclidean clustering (~50 lines) to avoid the dependency.

---

## Gazebo Simulation — People

**Cylinder person SDF** (`worlds/models/person_cylinder/model.sdf`):
```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="person_cylinder">
    <static>false</static>
    <link name="body">
      <pose>0 0 0.85 0 0 0</pose>
      <inertial>
        <mass>70.0</mass>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.15</radius><length>1.7</length></cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.15</radius><length>1.7</length></cylinder>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

**Moving people**: Use Gazebo's `/world/<world>/set_pose` service to teleport cylinders along a scripted path. Write a simple Python node that moves them at ~0.5 m/s behind the robot for testing.

---

## Testing Metrics (for Report)

Run each test scenario 5× and record:

| Metric | How to Measure |
|--------|---------------|
| Tour completion rate | % of runs where robot reaches final waypoint |
| Avg time per waypoint | Total tour time / waypoint count |
| Pause event count | Number of times FSM enters `PAUSED_WAITING` |
| Avg pause duration | Mean time spent in `PAUSED_WAITING` per event |
| Search event count | Times FSM enters `SEARCHING_FOR_GROUP` |
| Time-to-recovery | Time from `SEARCHING` entry to group re-detection |
| Collision count | Bump sensor triggers (should be 0) |
| Emergency stop count | `safety_monitor` activations |
| Detection accuracy | Compare `person_count` to ground truth over a run |

**Test scenarios**:
1. **Nominal**: 3 cylinders follow at ~2m, constant speed
2. **Slow followers**: Cylinders at 50% robot speed → triggers pausing
3. **Group lost**: Cylinders stop entirely → triggers search
4. **Obstacle in path**: Static box in Nav2's planned route
5. **Narrow passage**: 1.2m wide corridor with people behind

---

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|-----------|
| Nav2 config consumes too much time | Medium | High | Use simplified rectangular map first. Only attempt real SEC map if time allows. |
| Group detection too noisy | Medium | High | Fallback: timer-based leading (move 10s, pause 5s, repeat) — no detection needed. |
| twist_mux integration issues | Low | Medium | Test in Phase 1 day 1. Fallback: single `/cmd_vel` publisher in `tour_state_machine`, manually gate Nav2 output. |
| Sim-to-real gap breaks detection | High | Medium | Allocate full last week for real-world tuning. Have sim-only demo as backup. |
| Scope creep | Medium | Low | Cut camera fusion, narration, dynamic speed first. MVP is waypoints + pause/resume + e-stop. |

---

## External Dependencies & Attribution

Document in `attribution.md`:

| Package | License | What We Use |
|---------|---------|-------------|
| Nav2 | Apache 2.0 | Path planning, waypoint navigation, costmap, local planner |
| twist_mux (`twist_mux` ROS 2 pkg) | BSD | Velocity command priority multiplexing |
| TurtleBot 4 packages | Apache 2.0 | Robot description, sensor drivers, Gazebo sim |
| scikit-learn | BSD | DBSCAN clustering (in `group_tracker`) |
| Gazebo Harmonic | Apache 2.0 | Simulation environment |

**Original work** (must be clearly ours):
- `tour_state_machine.py` — FSM design and implementation
- `group_tracker.py` — rear-arc LiDAR clustering + tracking logic
- `safety_monitor.py` — emergency stop logic
- `clustering.py` — clustering utilities
- All config files, launch files, test scenarios, Gazebo world
- Tour waypoint design and threshold tuning

---

## Quick-Start Commands (for README)

```bash
# Build
cd sec_tour_guide_ws
colcon build --packages-select sec_tour_guide
source install/setup.bash

# Launch simulation tour
ros2 launch sec_tour_guide sim_tour.launch.py

# Launch detection tuning (no Nav2)
ros2 launch sec_tour_guide detection_test.launch.py

# Launch full system (real robot)
ros2 launch sec_tour_guide tour_guide.launch.py use_sim:=false

# Teleop override (separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key

# Monitor tour state
ros2 topic echo /tour/state

# Monitor group detection
ros2 topic echo /group_tracker/status
```
