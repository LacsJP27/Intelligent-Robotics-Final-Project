PROJECT 2

Ensure the Turtlebot is powered on

ssh into the Turtlebot
```
ssh student@<robot-name>.cs.nor.ou.edu
```

Check if /odem, /t, and /scan are present after checking the topic list
```
ros2 topic list
```

If they are not present restart the daemon then check the topic list again
```
turtlebot4-daemon-restart
```

In a new terminal run and in your ros2 workspace
```
robot-setup.sh
```
Then run the commands it returns

Build packages
```
colcon build --symlink-install
```

source ros2
```
source install/setup.bash
```

Run the launch file
```
 ros2 launch p2_world tb4_real.launch.py
```

To stop LiDAR:
```
ros2 service call /stop_motor std_srvs/srv/Empty "{}"
```

For keyboard control:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key
```
