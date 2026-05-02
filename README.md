# SEC Robot Tour Guide — Final Project Plan

> **Course**: CS 4023 — Intelligent Mobile Robotics (University of Oklahoma)
> **Demo Week**: May 4–8, 2026 | **Draft Report Due**: April 23
> **Team Size**: 3 | **Weight**: 50% of course grade
> **Platform**: TurtleBot 4 (Create 3 + RPi 4) · ROS 2 Jazzy · Gazebo Harmonic

---

## Mission Statement

Build a robot tour guide that leads people through waypoints inside SEC (Sarkeys Energy Center). The robot navigates ahead of a group, detects whether people are following via rear-arc LiDAR clustering, and adapts its behavior (pause, resume) accordingly.

---

## Minimum Viable Demo (MVP)

The MVP is the baseline that **must work** by demo day (May 4):

1. Robot navigates 3+ waypoints sequentially via Nav2 ✅
2. Emergency stop triggers when an obstacle is within 0.25m ✅
3. Human override via keyboard teleop at any time ✅
4. Robot pauses if no LiDAR clusters are detected behind it ⏳
5. Robot resumes navigation when clusters reappear ⏳

Items 1–3 are done (Phase 1). Items 4–5 are the focus of Phase 2 (group detection) and Phase 3 (FSM integration). Everything beyond this list (camera fusion, narration, dynamic speed modulation, real-world deployment) is a stretch goal — only attempted after the MVP runs end-to-end in sim.

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
│       │       ├── clustering.py          # DBSCAN + rear-arc filter (Phase 2)
│       │       └── constants.py           # Shared thresholds, topic names, frame IDs
│       │
│       ├── config/
│       │   └── tour_waypoints.yaml        # Waypoint sequence (x, y, yaw per stop)
│       │
│       ├── launch/
│       │   └── sim_tour.launch.py         # Gazebo sim launch (world + robot + people)
│       │
│       └── worlds/
│           └── models/
│               └── person_cylinder/       # Phase 2 follower model
│                   ├── model.config
│                   └── model.sdf          # Cylinder person model (r=0.15m, h=1.7m)
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
| `tour_state_machine.py` | FSM orchestrator. Owns `/cmd_vel`. Sends Nav2 goals, listens to group status and emergency flag, cancels Nav2 and publishes `Twist` directly when overriding. | `/cmd_vel` (`Twist`), `/tour/state` (`String`) | `/group/detected` (`Bool`), `/group/closest_distance` (`Float32`), `/emergency_stop` (`Bool`), `/cmd_vel_key` (`Twist`), Nav2 action feedback |
| `group_tracker.py` | Filters `/scan` to rear arc, runs DBSCAN clustering, publishes group detection status and closest distance. | `/group/detected` (`Bool`), `/group/closest_distance` (`Float32`) | `/scan` (`LaserScan`) |
| `safety_monitor.py` | Monitors `/scan` for emergency-close obstacles (<0.25m) in the forward arc. Publishes a simple Bool flag the FSM consumes. | `/emergency_stop` (`Bool`) | `/scan` (`LaserScan`) |

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

**Velocity command priority** is enforced by the FSM itself rather than a separate multiplexer node. The FSM owns `/cmd_vel`. When emergency or teleop fires, the FSM cancels the active Nav2 goal (so Nav2 stops publishing) and publishes the override `Twist` directly. Priority order, top to bottom: emergency stop → teleop → Nav2. We tried `twist_mux` early and removed it — for two override sources where the FSM already knows which mode it's in, an extra mux node adds package, config, and topic-remap surface without buying anything.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      DELIBERATIVE LAYER                      │
│                                                              │
│   ┌──────────────────────┐     ┌──────────────────────────┐ │
│   │  tour_state_machine  │────▶│  Nav2 (NavigateToPose)   │ │
│   │  (FSM, owns /cmd_vel)│◀────│  action server           │ │
│   └──────────┬───────────┘     └──────────────────────────┘ │
│              │   ▲   ▲                                       │
│              │   │   │                                       │
│              │   │   └── /cmd_vel_key (teleop_twist_keyboard)│
│              │   │                                           │
│              │   └────── /emergency_stop (Bool)              │
│              │                                               │
│              │ /group/detected + /group/closest_distance     │
│              │                                               │
├──────────────┼───────────────────────────────────────────────┤
│              │      REACTIVE LAYER                            │
│              │                                               │
│   ┌──────────┴───────────┐     ┌──────────────────────────┐ │
│   │   group_tracker      │     │   safety_monitor         │ │
│   │   (LiDAR clustering) │     │   (emergency stop flag)  │ │
│   └──────────▲───────────┘     └────────────▲─────────────┘ │
│              │                               │               │
├──────────────┼───────────────────────────────┼───────────────┤
│              │         HARDWARE / SENSORS    │               │
│              │                               │               │
│         /scan (RPLIDAR A1) ──────────────────┘               │
│              │                                               │
│         ┌────┴───────────────────────────────────────────┐   │
│         │         TurtleBot 4 (Create 3 + RPi 4)         │   │
│         └────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

The FSM is the only publisher on `/cmd_vel`. In `NAVIGATING` it sits silent and lets Nav2 publish; in any override state it cancels the Nav2 goal and publishes the override `Twist` itself.

---

## Tour State Machine

```
IDLE
  │  [operator starts tour via service call or topic]
  ▼
NAVIGATING
  ├─ [group_detected == false > 0.5s] ──▶ WAITING_FOR_GROUP
  ├─ [waypoint reached]               ──▶ WAYPOINT_REACHED
  └─ [emergency obstacle]             ──▶ EMERGENCY_STOP

WAITING_FOR_GROUP
  └─ [group_detected == true]         ──▶ NAVIGATING

WAYPOINT_REACHED
  ├─ [more waypoints remaining]       ──▶ NAVIGATING  (advance)
  └─ [last waypoint]                  ──▶ TOUR_COMPLETE

EMERGENCY_STOP
  └─ [obstacle cleared]               ──▶ NAVIGATING

TELEOP_OVERRIDE
  └─ [teleop released]                ──▶ NAVIGATING

TOUR_COMPLETE
  └─ [reset]                          ──▶ IDLE
```

Phase 1 implements all states except `WAITING_FOR_GROUP`. Phase 3 adds the `WAITING_FOR_GROUP` branch (pause when group not detected for 0.5s, resume when detected).

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

# --- Tour State Machine ---
GROUP_DETECT_THRESHOLD = 2.5    # meters — closest cluster triggers group_detected
GROUP_LOST_TIMEOUT = 0.5        # seconds — debounce before pause
WAYPOINT_DWELL_TIME = 2.0       # seconds — pause at each waypoint

# --- Safety ---
EMERGENCY_STOP_DISTANCE = 0.25  # meters — immediate stop

# --- Topic Names ---
TOPIC_SCAN = "/scan"
TOPIC_GROUP_DETECTED = "/group/detected"
TOPIC_GROUP_DISTANCE = "/group/closest_distance"
TOPIC_TOUR_STATE = "/tour/state"
TOPIC_EMERGENCY = "/emergency_stop"
TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_TELEOP = "/cmd_vel_key"
```

---

## Phased Implementation Plan

### Phase 1 — Foundation (Days 1–7): "Robot navigates waypoints" ✅ DONE

**Goal**: TurtleBot 4 autonomously navigates a sequence of waypoints in Gazebo using Nav2.

| Task | Owner | What Was Built |
|------|-------|----------------|
| Workspace scaffolding | All | `sec_tour_guide` package, `package.xml`, `setup.py`, folder structure, Git repo. |
| Map creation | **Member A** | SLAM-based map of the Gazebo corridor world. Nav2 localizes against it at launch. |
| Nav2 configuration | **Member A** | TurtleBot 4 Nav2 defaults used as-is. `NavigateToPose` action verified working. |
| `tour_state_machine` skeleton | **Member B** | FSM with states `IDLE → NAVIGATING → WAYPOINT_REACHED → TOUR_COMPLETE`, plus `EMERGENCY_STOP` and `TELEOP_OVERRIDE`. Loads waypoints from YAML, sends Nav2 goals one at a time. No group detection yet. |
| `safety_monitor` node | **Member C** | Subscribes to `/scan`, checks for any point < 0.25m in the forward arc, publishes `Bool` on `/emergency_stop`. |
| `sim_tour.launch.py` | **Member C** | Launch file: Gazebo world + robot + Nav2 + all custom nodes. |

**Phase 1 exit criteria**:
- [x] Robot spawns in Gazebo corridor world
- [x] Robot navigates to 3+ waypoints in sequence without collision
- [x] Teleop overrides Nav2 (FSM cancels goal and relays `/cmd_vel_key`)
- [x] Emergency stop triggers when object placed <0.25m in front

---

### Phase 2 — Group Detection (Days 5–14, overlaps Phase 1)

**Goal**: `group_tracker` node reliably detects simulated people behind the robot.

| Task | Owner | Details |
|------|-------|---------|
| `clustering.py` utilities | **Member B** | Implement: polar-to-cartesian conversion, DBSCAN wrapper. Fold any needed geometry helpers directly into this file. |
| `group_tracker` node | **Member B** | Subscribe `/scan` → filter rear arc → cluster → publish `/group/detected` (`Bool`) and `/group/closest_distance` (`Float32`). |
| Person cylinder model | **Member C** | Create SDF model: cylinder r=0.15m, h=1.7m, non-static. Place in `worlds/models/person_cylinder/`. |
| Gazebo people spawning | **Member C** | Script or launch file that spawns 2–3 person cylinders behind the robot. Simple Python node to move them at ~0.5 m/s via Gazebo's `set_entity_pose` service. |

**Phase 2 exit criteria**:
- [ ] `group_tracker` publishes `group_detected=true` when cylinders are 1–4m behind robot
- [ ] `group_tracker` publishes `group_detected=false` when no cylinders present
- [ ] `closest_distance` is within ±0.3m of ground truth

---

Running the spawn people script:
```
ros2 launch sec_tour_guide spawn_people.launch.py
```

### Phase 3 — Integration: Adaptive Leading (Days 12–21)

**Goal**: Full tour with pause/resume behavior driven by group detection.

| Task | Owner | Details |
|------|-------|---------|
| Add `WAITING_FOR_GROUP` state | **Member A** | Wire `/group/detected` and `/group/closest_distance` into `tour_state_machine`. On 0.5s group-lost debounce: `cancel_goal_async()`. On group re-detected: re-send current waypoint goal. |
| End-to-end Gazebo test | **Member B** | Test scenario: cylinders follow robot → cylinders stop (triggers pause) → cylinders resume (robot resumes). Tune DBSCAN params and distance threshold. Document final values. |

**Phase 3 exit criteria**:
- [ ] Robot pauses when group is not detected for 0.5s
- [ ] Robot resumes when group is detected again
- [ ] Full tour completes with no crashes or deadlocks

---

### Phase 4 — Polish, Real-World & Report (Days 19–28)

**Goal**: Demo-ready system, recorded data, written report.

| Task | Owner | Details |
|------|-------|---------|
| Real-world testing | **Member A** | Deploy to physical TurtleBot 4 in SEC. Tune thresholds for real human legs (noisier than cylinders). Record rosbag data. |
| Code cleanup | **All** | Docstrings on every class/method. Type hints. Consistent naming. Remove dead code. |
| `attribution.md` | **Member B** | Document all borrowed code: Nav2 (Apache 2.0), TurtleBot 4 packages, sklearn DBSCAN, Project 2 control pattern carry-over. Clearly separate original work. |
| Report — Architecture & Methods | **Member B** | System diagram, node descriptions, FSM design, detection algorithm explanation. |
| Report — Results & Testing | **Member C** | Run 5+ tours in simulation, record metrics. Create tables/graphs. Screenshots from Gazebo. |
| Report — Introduction & Conclusion | **Member A** | Problem statement, related work, lessons learned. |
| Demo video | **Member C** | Screen-record Gazebo demo. Optionally record real-world demo in SEC. |
| README.md | **Member A** | Build instructions, dependencies, quick-start guide, team member contributions. |

**Phase 4 exit criteria**:
- [ ] 5+ successful tour completions recorded (simulation)
- [ ] At least 1 real-world run attempted (even if imperfect)
- [ ] Report draft complete with all sections
- [ ] Demo video recorded
- [ ] `attribution.md` complete

---

## Team Member Assignment Summary

| Area | Member A | Member B | Member C |
|------|----------|----------|----------|
| **Phase 1** | Map + Nav2 config | FSM skeleton + waypoints | safety_monitor + launch |
| **Phase 2** | — | group_tracker + clustering | Gazebo people sim + cylinder models |
| **Phase 3** | FSM integration + Nav2 pause/resume | E2E testing + threshold tuning | — |
| **Phase 4** | Real-world testing + README + report intro | Attribution + report architecture | Report results + demo video |

**Parallel workstreams**: Phases 1 and 2 overlap (days 5–7). Member B can start `group_tracker` while A and C finish Nav2 setup. Phase 3 can begin as soon as Phase 1 core is working (Nav2 navigates waypoints) even if Phase 2 detection isn't perfect yet — use a mock `/group/detected` publisher for FSM development.

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

**Do NOT** run the existing obstacle avoidance node simultaneously with Nav2's local planner. Nav2 handles obstacle avoidance via its costmap. The `safety_monitor` only flags emergency-close obstacles via `/emergency_stop`; the FSM is the one that reacts (cancel Nav2 goal, publish zero `Twist`).

---

## Group Detection Algorithm

```
1. Receive LaserScan message
2. Filter to rear arc (120°–240° in base_link frame)
3. Filter by range (0.5m – 5.0m)
4. Convert remaining points: polar → Cartesian (x, y)
5. DBSCAN clustering (eps=0.3m, min_samples=3)
6. closest_distance = min(centroid distance) over remaining clusters
7. Publish /group/detected (Bool) + /group/closest_distance (Float32)
```

Width filter and motion-history classification deliberately deferred. Add only if testing reveals false positives.

**Dependencies**: `scikit-learn` for DBSCAN (`pip install scikit-learn` or `sudo apt install python3-sklearn`).

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
| Pause event count | Number of times FSM enters `WAITING_FOR_GROUP` |
| Detection accuracy | Compare `group_detected` to ground truth (cylinders present/absent) over a run |
| Collision count | Bump sensor triggers (should be 0) |

**Test scenarios**:
1. **Nominal**: 3 cylinders follow at ~2m, constant speed
2. **Slow followers**: Cylinders at 50% robot speed → triggers pausing
3. **Group lost**: Cylinders stop entirely → triggers pause
4. **Obstacle in path**: Static box in Nav2's planned route
5. **Narrow passage**: 1.2m wide corridor with people behind

---

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|-----------|
| Nav2 config consumes too much time | Medium | High | Use simplified rectangular map first. Only attempt real SEC map if time allows. |
| Group detection too noisy | Medium | High | Add the cluster-width filter (~5 lines) as first mitigation. Fallback: timer-based leading (move 10s, pause 5s, repeat) — no detection needed. |
| Nav2 keeps publishing `/cmd_vel` after override | Low | Medium | FSM cancels the Nav2 goal before publishing its own `Twist`; once a goal is cancelled, Nav2's controller stops emitting velocity. Verified in Phase 1 testing. |
| Sim-to-real gap breaks detection | High | Medium | Allocate full last week for real-world tuning. Have sim-only demo as backup. |
| Scope creep | Medium | Low | Cut camera fusion, narration, dynamic speed first. MVP is waypoints + pause/resume + e-stop. |

---

## External Dependencies & Attribution

Document in `attribution.md`:

| Package | License | What We Use |
|---------|---------|-------------|
| Nav2 | Apache 2.0 | Path planning, waypoint navigation, costmap, local planner |
| TurtleBot 4 packages | Apache 2.0 | Robot description, sensor drivers, Gazebo sim |
| scikit-learn | BSD | DBSCAN clustering (in `group_tracker`) |
| python3-sklearn (or pip) | BSD | DBSCAN clustering |
| Gazebo Harmonic | Apache 2.0 | Simulation environment |

**Original work** (must be clearly ours):
- `tour_state_machine.py` — FSM design and implementation
- `group_tracker.py` — rear-arc LiDAR clustering logic
- `safety_monitor.py` — emergency stop logic
- `clustering.py` — clustering utilities
- All config files, launch files, test scenarios, Gazebo world
- Tour waypoint design and threshold tuning

---

## Real-World Map Creation

Run these steps once to build and save the SEC map before running the full tour after an ssh into the turtlemap.

**MAKE SURE TO RUN THE `robot-setup.sh` COMMANDS ON EVERY NEW TERMINAL**

**Step 1 — Build and source**
```bash
cd ~/ros2_ws
colcon build --packages-select sec_tour_guide
source install/setup.bash
```

**Step 2 — Launch SLAM** (Terminal 1, keep open)
```bash
ros2 launch turtlebot4_navigation slam.launch.py \
  params:=$HOME/ros2_ws/install/sec_tour_guide/share/sec_tour_guide/config/slam_toolbox_params.yaml
```

**Step 3 — Open RViz to watch the map build** (Terminal 2)
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

**Step 4 — Launch teleop** (Terminal 3, keep open)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Step 5 — Drive around the entire room** covering all areas including corners and doorways.

**Step 6 — Save the map** (Terminal 4, once satisfied with coverage)
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/Intelligent-Robotics-Final-Project/src/sec_tour_guide/config/sec_real_map
```

This saves `sec_real_map.yaml` and `sec_real_map.pgm` into the config directory.

**Step 7 — Rebuild to install the new map files**
```bash
colcon build --packages-select sec_tour_guide
```

> **Note:** Make sure all terminals have the correct robot environment set:
> ```bash
> export ROS_DOMAIN_ID=2
> export ROS_DISCOVERY_SERVER=';;10.194.16.41:11811;'
> export ROS_SUPER_CLIENT=True
> unset ROS_LOCALHOST_ONLY
> ```

---

## Quick-Start Commands

```bash
# Build
cd sec_tour_guide_ws
colcon build --packages-select sec_tour_guide
source install/setup.bash

# Launch simulation tour
ros2 launch sec_tour_guide sim_tour.launch.py

# Teleop override (separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key

# Monitor tour state
ros2 topic echo /tour/state

# Monitor group detection
ros2 topic echo /group/detected
ros2 topic echo /group/closest_distance
```
