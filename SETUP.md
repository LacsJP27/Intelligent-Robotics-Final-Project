# Dev Environment Setup

## Python Dependencies (scikit-learn)

The group detection node (`group_tracker`) requires `scikit-learn`, which is not
available as a system apt package on the lab machines. The system Python also
blocks `pip install --user` via PEP 668.

### One-time setup

```bash
# 1. Create the virtual environment (only needed once)
cd ~/ros_ws
python3 -m venv ROB

# 2. Activate it and install scikit-learn
source ~/ros_ws/ROB/bin/activate
pip install scikit-learn

# 3. Deactivate — ros2 uses system Python, not the venv directly
deactivate

# 4. Add the venv's site-packages to PYTHONPATH permanently
echo 'export PYTHONPATH=~/ros_ws/ROB/lib/python3.12/site-packages:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

### Verify it worked

```bash
python3 -c "from sklearn.cluster import DBSCAN; print('ok')"
```

### Why this is needed

ROS 2 `ros2 run` uses the system Python interpreter, not a virtual environment.
The `PYTHONPATH` export tells the system Python to also search the venv's
site-packages, so it finds `sklearn` without needing admin access.

---

## Building the workspace

```bash
cd ~/ros_ws
colcon build --packages-select sec_tour_guide
source install/setup.bash
```

---

## Running the sim

**Terminal 1** — full simulation:
```bash
ros2 launch sec_tour_guide sim_tour.launch.py
```

**Terminal 2** — spawn and move the people:
```bash
ros2 launch sec_tour_guide spawn_people.launch.py
```

**Terminal 3** — monitor group detection:
```bash
ros2 topic echo /group/detected
```

**Terminal 4** — monitor FSM state:
```bash
ros2 topic echo /tour/state

** How to check muilple ros domain ids **
pgrep -af 'ros2'
pkill -f 'ros2cli.deamon'
```
