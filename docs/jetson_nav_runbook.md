# Jetson Indoor Navigation Runbook

This runbook is for on-robot validation of Guido's indoor LiDAR navigation stack.

## Baseline Checks

1. Source the ROS workspace and verify the required topics and TF frames exist.

```bash
source /opt/ros/humble/setup.bash
source ~/guido/install/setup.bash

ros2 topic list | grep -E '(/ldlidar_node/scan|/cmd_vel|/odom|/auto_nav/)'
ros2 run tf2_tools view_frames
```

2. Confirm the expected transform chain is available before autonomous tests:

- `map -> odom`
- `odom -> base_link`
- `base_link -> ldlidar_link`

3. If `/cmd_vel`, `/odom`, `/map`, or TF is missing, stop autonomous testing and fix the
robot base / mapping stack before continuing.

## Bringup Order

1. LiDAR only:

```bash
ros2 launch guido_bringup guido_lidar.launch.py
```

2. Mapping mode:

```bash
ros2 launch auto_nav mapping.launch.py
```

3. Save the map:

```bash
mkdir -p ~/.guido/maps
ros2 run nav2_map_server map_saver_cli -f ~/.guido/maps/<map_id>
```

4. Localization + navigation mode:

```bash
ros2 launch auto_nav navigation.launch.py \
  map_file:=~/.guido/maps/<map_id>.yaml \
  active_map_id:=<map_id>
```

5. Exploration mode:

```bash
ros2 launch auto_nav exploration.launch.py
```

## Waypoint Workflow

Save, inspect, and navigate to waypoints through `/auto_nav/command`:

```bash
ros2 topic pub --once /auto_nav/command std_msgs/msg/String "{data: 'save_waypoint couch'}"
ros2 topic pub --once /auto_nav/command std_msgs/msg/String "{data: 'list_waypoints'}"
ros2 topic pub --once /auto_nav/command std_msgs/msg/String "{data: 'navigate_to couch'}"
ros2 topic pub --once /auto_nav/command std_msgs/msg/String "{data: 'cancel_navigation'}"
ros2 topic pub --once /auto_nav/command std_msgs/msg/String "{data: 'stop'}"
```

Waypoints are stored in `~/.guido/waypoints.yaml` and are tied to the active map id.

## Exploration Workflow

Exploration mode is separate from saved-map waypoint navigation. Only
`start_exploration`, `cancel_navigation`, `stop`, and `help` are supported while
`exploration.launch.py` is running.

1. Use two terminals on your Mac. Do not `ssh` from inside the robot back into the same robot.
2. Terminal A: SSH in and prepare the launch shell.

```bash
ssh kwaii@172.20.10.5
cd ~/guido
source /opt/ros/humble/setup.bash
source ~/guido/install/setup.bash
mkdir -p ~/.guido/maps
```

3. If `auto_nav` still is not launchable in Terminal A, rebuild once:

```bash
cd ~/guido
source /opt/ros/humble/setup.bash
colcon build --packages-select auto_nav --symlink-install
source ~/guido/install/setup.bash
```

4. Terminal A: launch frontier mapping.

```bash
ros2 launch auto_nav exploration.launch.py
```

5. Terminal B: open a second SSH session and source the environment again.

```bash
ssh kwaii@172.20.10.5
cd ~/guido
source /opt/ros/humble/setup.bash
source ~/guido/install/setup.bash
```

6. Terminal B: confirm `/ldlidar_node/scan`, `/cmd_vel`, `/odom`, `/map`, and TF are all live.

```bash
ros2 topic echo --once /ldlidar_node/scan
ros2 topic echo --once /map
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /cmd_vel -v
ros2 topic info /auto_nav/frontier_markers
```

7. Terminal B: start exploration:

```bash
ros2 topic pub --once /auto_nav/command std_msgs/msg/String "{data: 'start_exploration'}"
```

8. Terminal B: monitor runtime state:

```bash
ros2 topic echo /auto_nav/status --field data
```

9. Robot display: open RViz with the exploration preset.

```bash
source /opt/ros/humble/setup.bash
source ~/guido/install/setup.bash
rviz2 -d $(ros2 pkg prefix auto_nav)/share/auto_nav/config/exploration.rviz
```

Successful visualization looks like this:
- Orange frontier clusters hug the unknown/free boundary.
- The blue frontier goal arrow sits on reachable free space near the selected frontier.
- The cyan line runs from the robot pose to the active goal.
- Red discs appear over blacklisted frontier regions after failures.

10. Use `stop` as the emergency software stop:

```bash
ros2 topic pub --once /auto_nav/command std_msgs/msg/String "{data: 'stop'}"
```

11. Cancel the active frontier and exit exploration mode:

```bash
ros2 topic pub --once /auto_nav/command std_msgs/msg/String "{data: 'cancel_navigation'}"
```

On successful completion, the command node saves the map to
`~/.guido/maps/explore_<timestamp>` and uses that saved stem as the new active map id for
later waypoint compatibility.

### Troubleshooting

- `Package 'auto_nav' not found`:
  - Source both environments again:
    - `source /opt/ros/humble/setup.bash`
    - `source ~/guido/install/setup.bash`
  - If needed, rebuild:
    - `colcon build --packages-select auto_nav --symlink-install`
- If you accidentally run `ssh kwaii@172.20.10.5` from inside the robot:
  - Exit that shell and open a second terminal on your Mac instead.
- If `ros2 topic echo /auto_nav/status` is noisy:
  - Use `ros2 topic echo /auto_nav/status --field data`
- The `RTPS_TRANSPORT_SHM Error` lines are noisy Fast DDS shared-memory warnings:
  - If topics, TF, and nodes are otherwise working, they are usually not the primary blocker.
- If goals are accepted but the robot does not move:
  - Watch the launch terminal for planner warnings like `failed to generate a valid path`.
  - Open the exploration RViz preset and verify the goal arrow is on reachable free space near
    the frontier edge, not buried in unknown cells.
  - If the markers look correct but the robot still does not translate, exploration orchestration
    is running but real-world driving is still not validated.

## Acceptance Checklist

- LiDAR publishes stable scans on `/ldlidar_node/scan`.
- Localization stays stable after loading a saved map.
- Three or more saved waypoints can be listed and revisited on the same map.
- The chair slows, stops, or replans when a static or dynamic obstacle blocks the path.
- `stop` and `cancel_navigation` halt autonomous movement safely.
- New goals are rejected when scan, odom, or TF health goes stale.
- Exploration only starts when `/cmd_vel`, `/odom`, `/map`, and TF are all available.
- Exploration generates and executes live frontier goals from reachable free-space poses,
  blacklists failed frontiers, publishes `/auto_nav/frontier_markers`, and saves the completed
  map to `~/.guido/maps/explore_<timestamp>`.

## Shutdown

Stop the robot before shutting down the stack:

```bash
ros2 topic pub --once /auto_nav/command std_msgs/msg/String "{data: 'stop'}"
```

Then terminate the active launch process and power down the motion system if needed.
