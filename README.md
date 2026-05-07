# SEC Robot Tour Guide

> **Course**: CS 4023 — Intelligent Mobile Robotics (University of Oklahoma)
> **Platform**: TurtleBot 4 · ROS 2 Jazzy · Gazebo Harmonic

---

## Overview

The SEC Robot Tour Guide is an autonomous mobile robot that leads visitors through waypoints inside Sarkeys Energy Center (SEC). The robot uses Nav2 for path planning and navigates ahead of a group. A rear-facing LiDAR clustering algorithm detects whether people are following; if the group falls behind, the robot pauses and waits before resuming. An emergency stop halts the robot immediately if any obstacle comes within a configurable threshold.

The system runs in both Gazebo simulation and on a physical TurtleBot 4.

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
│              │   │   └── /cmd_vel_key (teleop_twist_keyboard)│
│              │   └────── /emergency_stop (Bool)              │
│              │ /group/detected (Bool)                        │
│              │                                               │
├──────────────┼───────────────────────────────────────────────┤
│              │           REACTIVE LAYER                      │
│              │                                               │
│   ┌──────────┴───────────┐     ┌──────────────────────────┐ │
│   │   group_tracker      │     │   safety_monitor         │ │
│   │   (LiDAR clustering) │     │   (emergency stop flag)  │ │
│   └──────────▲───────────┘     └────────────▲─────────────┘ │
│              │                               │               │
├──────────────┼───────────────────────────────┼───────────────┤
│              │        HARDWARE / SENSORS     │               │
│              └──────── /scan (RPLiDAR) ──────┘               │
│                                                              │
│         ┌──────────────────────────────────────────────┐    │
│         │                 TurtleBot 4                  │    │
│         └──────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### Topics

| Topic             | Type                     | Producer                | Consumer(s)                       |
| ----------------- | ------------------------ | ----------------------- | --------------------------------- |
| `/scan`           | `sensor_msgs/LaserScan`  | TurtleBot 4 LiDAR       | `group_tracker`, `safety_monitor` |
| `/cmd_vel`        | `Twist` / `TwistStamped` | `tour_state_machine`    | TurtleBot 4 base                  |
| `/cmd_vel_key`    | `Twist` / `TwistStamped` | `teleop_twist_keyboard` | `tour_state_machine`              |
| `/emergency_stop` | `std_msgs/Bool`          | `safety_monitor`        | `tour_state_machine`              |
| `/group/detected` | `std_msgs/Bool`          | `group_tracker`         | `tour_state_machine`              |
| `/tour/state`     | `std_msgs/String`        | `tour_state_machine`    | monitoring                        |

The FSM is the **sole publisher** on `/cmd_vel`. In `NAVIGATING` it lets Nav2 drive; in any override state it cancels the active Nav2 goal and publishes velocity commands directly.

---

## Tour State Machine

```
IDLE
  │  [Nav2 ready + waypoints loaded]
  ▼
NAVIGATING
  ├─ [group not detected > 0.5 s]  ──▶ WAITING_FOR_GROUP
  ├─ [waypoint reached]           ──▶ WAYPOINT_REACHED
  ├─ [obstacle < 0.05 m forward]  ──▶ EMERGENCY_STOP
  └─ [teleop key pressed]         ──▶ TELEOP_OVERRIDE

WAITING_FOR_GROUP
  └─ [group re-detected]          ──▶ NAVIGATING  (re-sends same waypoint)

WAYPOINT_REACHED
  ├─ [more waypoints]             ──▶ NAVIGATING  (2 s dwell, then advance)
  └─ [last waypoint]              ──▶ TOUR_COMPLETE

EMERGENCY_STOP
  └─ [obstacle cleared]           ──▶ NAVIGATING

TELEOP_OVERRIDE
  └─ [no key for 0.5 s]           ──▶ NAVIGATING

TOUR_COMPLETE
  └─ (publishes zero velocity; stays here until process restart)
```

Override priority (highest → lowest): `EMERGENCY_STOP` → `TELEOP_OVERRIDE` → Nav2.

---

## Group Detection Algorithm

`group_tracker` runs on every `/scan` message:

1. Filter LiDAR returns to the **rear arc** (70°–110° in `base_link` frame)
2. Keep only points in range 0.5 m – 5.0 m
3. Convert polar → Cartesian (x, y)
4. Run **DBSCAN** (ε = 0.3 m, min_samples = 3) to find clusters
5. Compute distance to nearest cluster centroid
6. Publish `group/detected = True` if that distance ≤ 1.0 m

---

## Dependencies

| Dependency           | Version  | Notes                                                 |
| -------------------- | -------- | ----------------------------------------------------- |
| ROS 2                | Jazzy    |                                                       |
| Nav2                 | Jazzy    | Path planning, costmap, action server                 |
| TurtleBot 4 packages | Jazzy    | Robot description, drivers, Gazebo sim                |
| Gazebo               | Harmonic | Simulation only                                       |
| scikit-learn         | ≥ 1.0    | DBSCAN for group detection — see [SETUP.md](SETUP.md) |

**scikit-learn is not available as a system apt package on lab machines.** Follow [SETUP.md](SETUP.md) for the one-time venv + `PYTHONPATH` setup required before running.

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select sec_tour_guide
source install/setup.bash
```

---

## Running

### Simulation

**Terminal 1** — launch Gazebo + Nav2 + all custom nodes:

```bash
ros2 launch sec_tour_guide sim_tour.launch.py
```

**Terminal 2** — spawn and move simulated people (optional, needed for group detection):

```bash
ros2 launch sec_tour_guide spawn_people.launch.py
```

**Terminal 3** — teleop override (optional):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_key
```

**Monitor state** (any terminal):

```bash
ros2 topic echo /tour/state
ros2 topic echo /group/detected
```

---

### Real Robot

> **Every new terminal must source the robot environment first:**
>
> ```bash
> robot-setup.sh
> ```

**Terminal 1** — launch localization, Nav2, RViz, and custom nodes:

```bash
ros2 launch sec_tour_guide real_tour.launch.py
```

The FSM starts automatically 30 seconds after launch (delay lets Nav2 initialize). Teleop override is available at any time via `teleop_twist_keyboard` on `/cmd_vel_key`.

---

#### Building a New Real-World Map (SLAM)

Run this once to create a map before using `real_tour.launch.py`.

**Terminal 1** — SLAM:

```bash
ros2 launch turtlebot4_navigation slam.launch.py \
  params_file:=$HOME/ros_ws/install/sec_tour_guide/share/sec_tour_guide/config/slam_toolbox_params.yaml
```

**Terminal 2** — RViz (watch the map build):

```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

**Terminal 3** — teleop to drive around the space:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

**Terminal 4** — save the map when satisfied with coverage:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros_ws/src/Intelligent-Robotics-Final-Project/src/sec_tour_guide/config/{YOUR_MAPNAME}
```

Then rebuild so the installed share directory picks up the new map files:

```bash
colcon build --packages-select sec_tour_guide
```

---

## Configuration

### Waypoints (`config/tour_waypoints.yaml`)

```yaml
tour:
  frame_id: 'map'
  waypoints:
    # Active set — south robotics lab
    - { x: -12.237, y: 7.086, yaw: 0.0, label: 'Stop 1 — straight short' }
    - { x: -5.035, y: 8.481, yaw: 0.0, label: 'Stop 2 — straight long' }
```

Add, remove, or reorder `waypoints` entries to change the tour route. Coordinates are in the `map` frame; `yaw` is in radians (0 = +x, π/2 = +y). The file contains commented-out waypoint sets for the open real map and the SEC map — swap the active block when switching environments. The matching initial pose must also be updated in `nav2_params.yaml` under `amcl > initial_pose`; reference coordinates for each map are in the corresponding `*_coords.txt` file.

### Key Parameters (`utils/constants.py`)

| Constant                   | Default | Description                                            |
| -------------------------- | ------- | ------------------------------------------------------ |
| `EMERGENCY_STOP_DISTANCE`  | 0.05 m  | Forward obstacle distance that triggers emergency stop |
| `FORWARD_ARC_HALF_DEG`     | 90°     | Half-width of the forward danger cone                  |
| `GROUP_LOST_TIMEOUT_SEC`   | 0.5 s   | How long without group detection before pausing        |
| `GROUP_THRESHOLD_DISTANCE` | 1.0 m   | Max centroid distance to count as "group detected"     |
| `WAYPOINT_DWELL_TIME`      | 2.0 s   | Pause duration at each waypoint before advancing       |
| `TELEOP_TIMEOUT_SEC`       | 0.5 s   | Inactivity window before teleop override releases      |

### Simulation vs. Real Robot: `cmd_vel` Message Type

The Create 3 base requires `geometry_msgs/TwistStamped`; the Gazebo diff-drive plugin requires plain `geometry_msgs/Twist`. The FSM reads a `use_stamped_cmd_vel` parameter — set automatically by each launch file (`True` in `real_tour.launch.py`, omitted/`False` in `sim_tour.launch.py`).

Nav2 also needs matching configuration. Three lines in `config/nav2_params.yaml` control this:

| Block               | Parameter                | Sim value | Real value |
| ------------------- | ------------------------ | --------- | ---------- |
| `controller_server` | `enable_stamped_cmd_vel` | `false`   | `true`     |
| `velocity_smoother` | `enable_stamped_cmd_vel` | `false`   | `true`     |
| `collision_monitor` | `enable_stamped_cmd_vel` | `false`   | `true`     |

**The committed state is configured for the real robot.** Before running the simulation, set all three to `false`, then rebuild.

Verify after launching:

```bash
ros2 topic info /cmd_vel -v
# Sim:  Type: geometry_msgs/msg/Twist
# Real: Type: geometry_msgs/msg/TwistStamped
```

---

## External Dependencies & Attribution

| Package              | License    | Usage                                                |
| -------------------- | ---------- | ---------------------------------------------------- |
| Nav2                 | Apache 2.0 | Path planning, costmap, local planner, action server |
| TurtleBot 4 packages | Apache 2.0 | Robot description, sensor drivers, Gazebo simulation |
| Gazebo Harmonic      | Apache 2.0 | Simulation environment                               |
| scikit-learn         | BSD        | DBSCAN clustering in `group_tracker`                 |

**Original work** (written by this team):

- `tour_state_machine.py` — FSM design and implementation
- `group_tracker.py` — rear-arc LiDAR clustering logic
- `safety_monitor.py` — forward-arc emergency stop
- `group_spawner.py` — Gazebo cylinder spawner and mover
- `utils/clustering.py` — DBSCAN pipeline helpers
- All launch files, waypoint definitions, and the Gazebo world
