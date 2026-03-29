# Guido Autonomous Navigation Stack

This repository now contains a ROS 2 Humble-oriented LiDAR navigation stack for a smart-car prototype built on Guido's current differential-drive hardware. The implementation is intentionally conservative: it is optimized for safe real-time operation on a Jetson Orin Nano, modular debugging, and a clean path to Ackermann scaling later.

## 1. System Architecture

### Text Diagram

```text
                           +----------------------------------+
                           |        Voice / Operator UI       |
                           | voice_stream + ADK + RViz goal   |
                           +----------------+-----------------+
                                            |
                                            v
                         /voice/command_text or /goal_pose
                                            |
                                            v
                         +------------------+------------------+
                         |        guido_mission_manager        |
                         | waypoint lookup + mode switching    |
                         +------------------+------------------+
                                            |
                    +-----------------------+-----------------------+
                    |                                               |
                    v                                               v
            /navigation/goal                                  /system/mode
                    |                                               |
                    v                                               |
    +---------------+---------------+                               |
    |        guido_global_planner   |                               |
    | Hybrid A* with dynamic overlay|                               |
    +---------------+---------------+                               |
                    |                                               |
                    v                                               |
         /navigation/global_path                                    |
                    |                                               |
                    v                                               |
    +---------------+---------------+                               |
    |    guido_trajectory_controller|                               |
    | pure pursuit + speed shaping  |                               |
    +---------------+---------------+                               |
                    |                                               |
                    v                                               |
              /cmd_vel_autonomy                                     |
                                                                    |
     /cmd_vel_manual --------------------+                          |
                                         v                          |
                             +-----------+-----------+              |
                             |     guido_command_mux |<-------------+
                             | manual / auto / hold  |
                             +-----------+-----------+
                                         |
                                         v
                                 /cmd_vel_request
                                         |
                                         v
                             +-----------+-----------+
                             |   guido_safety_monitor|
                             | E-stop + watchdogs    |
                             +-----------+-----------+
                                         |
                                         v
                                     /cmd_vel
                                         |
                                         v
                             +-----------+-----------+
                             | guido_serial_bridge   |
                             | serial <-> Arduino    |
                             +-----+-------------+---+
                                   |             |
                             /odom + TF     /imu/data_raw
                                   |             |
                                   v             v
                             +-----------+-------------+
                             |   robot_localization    |
                             |        EKF fusion       |
                             +-----------+-------------+
                                         |
                                 /odometry/filtered
                                         |
                                         v
                            +------------+------------+
                            | guido_localization_bridge|
                            | TF -> /navigation/pose   |
                            +------------+------------+
                                         ^
                                         |
              +--------------------------+--------------------------+
              |                                                     |
              v                                                     v
      Cartographer / Mapping                                  AMCL + map_server
      (map creation phase)                                    (runtime localization)

 LiDAR path:
 /ldlidar_node/scan -> guido_lidar_perception -> point cloud + local costmap + tracked obstacles
```

### Why This Architecture

- `guido_lidar_perception` converts raw scan data into data products the planner and controller can use immediately: point cloud, local costmap, and tracked obstacle markers.
- `robot_localization` + `guido_localization_bridge` separate raw base odometry from fused navigation pose. That makes the stack more stable at speed and lets AMCL or another SLAM backend own the `map -> odom` correction.
- `guido_global_planner` handles non-holonomic route generation with heading-aware search instead of point-to-point A* only.
- `guido_trajectory_controller` generates smooth motion and speed shaping instead of bang-bang waypoint hopping.
- `guido_safety_monitor` is independent of the planner and can stop the robot even if the planner/controller produce bad commands.
- `guido_command_mux` keeps manual override first-class. The system can drop into `manual` or `hold` without touching the low-level bridge.

## 2. ROS 2 Node Structure

| Node | Package | Purpose | Key I/O |
|------|---------|---------|---------|
| `guido_serial_bridge` | `guido_base` | `/cmd_vel` to Arduino, publishes odom + IMU | `/cmd_vel`, `/odom`, `/imu/data_raw` |
| `guido_lidar_perception` | `guido_navigation` | LiDAR -> point cloud, local costmap, tracked obstacles | `/ldlidar_node/scan`, `/navigation/points`, `/navigation/local_costmap` |
| `ekf_filter_node` | `robot_localization` | Fuses wheel odom and IMU | `/odom`, `/imu/data_raw`, `/odometry/filtered` |
| `amcl` | `nav2_amcl` | Global localization on saved map | `/map`, `/scan`, TF |
| `guido_localization_bridge` | `guido_navigation` | Publishes unified navigation pose from TF | TF -> `/navigation/pose`, `/navigation/localization_ok` |
| `guido_mission_manager` | `guido_navigation` | Voice + RViz goal intake, waypoint resolution | `/voice/command_text`, `/goal_pose`, `/navigation/goal`, `/system/mode` |
| `guido_global_planner` | `guido_navigation` | Hybrid-A* global routing with dynamic obstacle overlay | `/map`, `/navigation/pose`, `/navigation/goal`, `/navigation/global_path` |
| `guido_trajectory_controller` | `guido_navigation` | Smooth local path following | `/navigation/global_path`, `/navigation/pose`, `/navigation/local_costmap`, `/cmd_vel_autonomy` |
| `guido_command_mux` | `guido_navigation` | Chooses manual, auto, or hold command source | `/cmd_vel_manual`, `/cmd_vel_autonomy`, `/system/mode`, `/cmd_vel_request` |
| `guido_safety_monitor` | `guido_navigation` | Final command gate, stopping-distance safety, watchdogs | `/cmd_vel_request`, `/ldlidar_node/scan`, `/odometry/filtered`, `/cmd_vel` |

## 3. Topic / Message Flow

| Topic | Type | Producer | Consumer |
|------|------|----------|----------|
| `/ldlidar_node/scan` | `sensor_msgs/msg/LaserScan` | LiDAR driver | perception, AMCL, safety |
| `/navigation/points` | `sensor_msgs/msg/PointCloud2` | perception | planner |
| `/navigation/local_costmap` | `nav_msgs/msg/OccupancyGrid` | perception | controller |
| `/navigation/obstacle_markers` | `visualization_msgs/msg/MarkerArray` | perception | RViz |
| `/navigation/tracking_markers` | `visualization_msgs/msg/MarkerArray` | perception | RViz |
| `/odom` | `nav_msgs/msg/Odometry` | serial bridge | EKF |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | serial bridge | EKF |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF | safety, controller |
| `/navigation/pose` | `geometry_msgs/msg/PoseStamped` | localization bridge | planner, controller |
| `/navigation/localization_ok` | `std_msgs/msg/Bool` | localization bridge | safety |
| `/voice/command_text` | `std_msgs/msg/String` | ADK transcript bridge or manual pub | mission manager |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz | mission manager |
| `/navigation/goal` | `geometry_msgs/msg/PoseStamped` | mission manager | planner |
| `/navigation/global_path` | `nav_msgs/msg/Path` | planner | controller, RViz |
| `/cmd_vel_manual` | `geometry_msgs/msg/Twist` | keyboard teleop | command mux |
| `/cmd_vel_autonomy` | `geometry_msgs/msg/Twist` | trajectory controller | command mux |
| `/cmd_vel_request` | `geometry_msgs/msg/Twist` | command mux | safety |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | safety | serial bridge |
| `/navigation/emergency_stop` | `std_msgs/msg/Bool` | safety | command mux, diagnostics |

## 4. Planning and Control Design

### Global Planner: Hybrid A*

Why:
- Standard A* ignores heading and turning radius, which produces paths that are fine for a point robot but not for a car-like platform.
- Hybrid A* lets us reason over `(x, y, yaw)` with steering primitives, so the route is already smooth enough to track.

What this implementation does:
- Discretizes heading into bins.
- Expands forward motion primitives with steering samples.
- Optionally supports reverse motion later with a parameter switch.
- Overlays current LiDAR obstacles on top of the static map before planning.
- Smooths the resulting path to reduce curvature spikes.

### Local Planner / Controller: Regulated Pure Pursuit

Why:
- It is light enough for the Orin Nano.
- It respects forward vehicle motion and avoids point-to-point jerk.
- It is easier to tune than MPC on a fast-moving prototype while still giving smooth trajectories.

What this implementation does:
- Computes a speed-dependent lookahead target on the global path.
- Converts lateral error into curvature.
- Reduces speed for:
  - high curvature
  - nearby local obstacles
  - goal approach
- Applies acceleration and deceleration limits before publishing commands.

### Dynamic Obstacle Handling

This stack uses three layers:

1. Perception:
   - clusters scan points
   - tracks centroids frame-to-frame
   - flags moving objects in RViz

2. Planner:
   - overlays near-field LiDAR points onto the global occupancy map before each plan cycle

3. Safety:
   - computes stopping distance in real time from current/requested speed
   - performs hard stop if anything enters the braking corridor

That layered approach is more robust than trusting one planner to do everything.

## 5. Pseudocode

### LiDAR Processing

```text
on_scan(scan):
  points = []
  for range sample in scan:
    if valid(sample):
      points.append(polar_to_xy(sample, angle))

  local_grid = unknown_grid()
  for point in points:
    raytrace(base, point) -> mark free
    mark occupied at obstacle cell
  inflate occupied cells

  clusters = cluster_adjacent_points(points)
  tracks = nearest_neighbor_track_update(clusters)

  publish point cloud
  publish local occupancy grid
  publish obstacle + tracking markers
```

### Planning Pipeline

```text
planner_timer():
  if map, pose, goal not ready:
    return

  overlay = static_map.copy()
  for lidar point in local_point_cloud:
    transform point from base frame to map frame
    mark nearby overlay cells occupied

  path = hybrid_a_star(
    start=(x, y, yaw),
    goal=(x, y, yaw),
    motion_primitives=[steer_left, steer_mid_left, straight, ...],
    collision_checker=overlay + footprint
  )

  if path found:
    smooth path
    publish nav_msgs/Path
  else:
    publish planner failure status
```

### Control Loop

```text
control_timer():
  if no pose or no path:
    publish zero cmd
    return

  target = first path point beyond dynamic lookahead distance
  transform target into vehicle frame
  curvature = 2 * y_local / lookahead^2

  speed = max_speed
  speed *= curvature_speed_factor(curvature)
  speed *= local_obstacle_factor(costmap)
  speed *= goal_slowdown_factor(distance_to_goal)
  speed = apply_acceleration_limits(speed)

  cmd.linear.x = speed
  cmd.angular.z = clamp(speed * curvature)
  publish cmd
```

## 6. Example ROS 2 Python Code

### LiDAR Subscriber Example

```python
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

class ScanPrinter(Node):
    def __init__(self):
        super().__init__('scan_printer')
        self.create_subscription(LaserScan, '/ldlidar_node/scan', self.cb, 10)

    def cb(self, msg: LaserScan):
        self.get_logger().info(f'samples={len(msg.ranges)} frame={msg.header.frame_id}')
```

### Velocity Publisher Example

```python
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)

    def send_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.25
        self.pub.publish(cmd)
```

The real repository implementations are:
- `src/guido_navigation/guido_navigation/lidar_perception.py`
- `src/guido_navigation/guido_navigation/trajectory_controller.py`
- `src/guido_navigation/guido_navigation/safety_monitor.py`

## 7. Arduino Motor Control Contract

Arduino responsibilities:
- parse `L<int> R<int>` and `STOP`
- enforce a serial command watchdog
- ramp PWM to avoid drivetrain shocks
- publish signed wheel odometry
- publish raw IMU data for EKF fusion

Serial protocol:

```text
Jetson -> Arduino:
  L120 R118
  STOP

Arduino -> Jetson:
  ODOM <x> <y> <theta> <v_lin> <v_ang>
  IMU <ax> <ay> <az> <gx> <gy> <gz>
```

Relevant files:
- `firmware/wheelchair_arduino.ino`
- `src/guido_base/guido_base/serial_bridge.py`

## 8. Runtime Modes

### Manual

- `guido_command_mux` selects `/cmd_vel_manual`
- safety monitor still enforces LiDAR stop distance
- localization loss does not block manual recovery

### Auto

- mission manager or RViz goal sets mode to `auto`
- global planner + local controller drive the robot
- safety monitor requires live localization and fresh scan/odom

### Hold

- zero-velocity mode
- entered by stop command, emergency stop, or explicit hold request

## 9. Jetson Bringup

### Install ROS 2 Humble

```bash
cd ~/guido
chmod +x scripts/install_ros2_humble.sh
sudo ./scripts/install_ros2_humble.sh
source /opt/ros/humble/setup.bash
```

### Build the Workspace

```bash
cd ~/guido
chmod +x scripts/setup_lidar.sh
./scripts/setup_lidar.sh
source install/local_setup.bash
```

### Launch the Autonomous Stack on the Jetson

RViz disabled on the Jetson:

```bash
ros2 launch guido_bringup guido_autonomy.launch.py \
  use_amcl:=true \
  map_yaml:=/home/kwaii/maps/guido_map.yaml \
  use_rviz:=false
```

RViz enabled on the Jetson:

```bash
ros2 launch guido_bringup guido_autonomy.launch.py \
  use_amcl:=true \
  map_yaml:=/home/kwaii/maps/guido_map.yaml \
  use_rviz:=true
```

### Manual Override with WASD

Open a second terminal on the Jetson:

```bash
cd ~/guido
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run guido_base keyboard_teleop --ros-args -p cmd_topic:=/cmd_vel_manual
```

### Voice Commands into ROS

```bash
cd ~/guido
source /opt/ros/humble/setup.bash
source install/local_setup.bash

python3 scripts/voice_stream.py --model models/vosk-model-small-en-us-0.15 \
  | python3 scripts/adk_transcript_bridge.py --ros-topic /voice/command_text
```

### Send a Goal from CLI

```bash
ros2 topic pub --once /voice/command_text std_msgs/msg/String "{data: 'go to parking lot'}"
```

### Stop Immediately

```bash
ros2 topic pub --once /voice/command_text std_msgs/msg/String "{data: 'stop'}"
```

## 10. Laptop RViz Workflow

Run the heavy compute on the Jetson and RViz on your laptop.

On both machines:

```bash
export ROS_DOMAIN_ID=42
```

On the Jetson:

```bash
ros2 launch guido_bringup guido_autonomy.launch.py \
  use_amcl:=true \
  map_yaml:=/home/kwaii/maps/guido_map.yaml \
  use_rviz:=false
```

On your laptop:

```bash
source /opt/ros/humble/setup.bash
rviz2
```

If you cloned the repo on the laptop too, you can load the included RViz config:

```bash
rviz2 -d /absolute/path/to/guido/src/guido_bringup/rviz/guido_navigation.rviz
```

## 11. Safety Strategy

- Emergency braking is independent of planning.
- Planner failure does not directly command the motors.
- Loss of LiDAR, odometry, or localization in auto mode results in stop.
- Manual override stays available even if map localization drops out.
- The Arduino enforces a serial watchdog so stale Jetson commands do not continue driving.
- Acceleration ramping prevents sharp PWM steps that can destabilize the platform.

## 12. Scaling to Ackermann

The current implementation targets the existing differential-drive prototype, but the architecture is intentionally car-ready:

- keep Hybrid A* and heading-aware planning
- replace `cmd_vel` actuator adapter with steering-angle + speed adapter
- swap the pure-pursuit angular command output for steering command output
- update footprint and turning-radius parameters
- keep the same perception, mission, localization, and safety layers

That means the software stack can grow into a smarter car platform without throwing away the safety architecture.
