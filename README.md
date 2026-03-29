# Guido Smart Wheelchair

ROS 2 wheelchair bringup and autonomous navigation stack for a Jetson + Arduino platform with LD19 LiDAR, serial motor control, waypointing, and Nav2 path following (manual teleop / WASD builds the map).

## System Architecture Summary

### Current low-level motion path

1. Any ROS publisher sends `geometry_msgs/msg/Twist` to `/cmd_vel`
2. `guido_serial_bridge` converts `Twist` into left/right PWM commands
3. The bridge sends ASCII serial packets to the Arduino:
   - `L<int> R<int>\n`
   - `STOP\n`
4. The Arduino drives the H-bridge / motor controller pins
5. The Arduino publishes odometry back to the Jetson over serial:
   - `ODOM x y theta linearVel angularVel`
6. `guido_serial_bridge` republishes `/odom` and `odom -> base_footprint`

### Current sensing path

1. The LD19 LiDAR is expected to publish scans through `ldlidar_node`
2. The scan topic is `/ldlidar_node/scan`
3. `slam_toolbox` consumes that scan and builds `/map`
4. Nav2 consumes `/map`, `/odom`, and `/ldlidar_node/scan`

### Recovered autonomy path

1. A user or bridge publishes plain-text commands to `/auto_nav/command`
2. `auto_nav_command_node` validates health, TF, and commands
3. The command node emits JSON navigation requests on `/auto_nav/request`
4. `auto_nav_navigation_node` forwards those requests to Nav2 `navigate_to_pose`
5. Nav2 publishes `/cmd_vel`
6. The existing serial bridge and Arduino move the chair

### Voice path

1. `scripts/voice_stream.py` converts microphone audio to transcript lines
2. `scripts/adk_transcript_bridge.py` normalizes transcripts and optionally:
   - uses Google ADK to resolve live saved waypoint names
   - publishes safe commands into `/auto_nav/command`
3. The voice stack never publishes raw motor commands

## Existing Components And Status

| Component | Status | Notes |
|-----------|--------|-------|
| `src/guido_base` | Implemented | Serial bridge and keyboard teleop |
| `src/guido_bringup` | Implemented | Base and LiDAR launch, URDF, LiDAR config |
| `firmware/wheelchair_arduino.ino` | Implemented, hardened | Added Arduino-side watchdog and signed odometry estimate |
| `src/auto_nav` | Reconstructed | Waypoint supervisor, ROS bridge nodes, launch/config, tests |
| `scripts/voice_stream.py` | Implemented | Offline Vosk or ElevenLabs STT |
| `scripts/adk_transcript_bridge.py` | Extended | Can now publish safe ROS commands |
| `agents/guido_mission_agent` | Implemented | ADK now reads the live `auto_nav` waypoint store and queues safe ROS waypoint commands |
| `frontend/guido-live-view` | Implemented | React + Tailwind live operations surface for map, path, pose, scan, and saved places |
| `src/ldrobot-lidar-ros2` | Missing in this checkout | Must initialize the git submodule before LiDAR bringup works |

What is still not present in the repo:

- Google Maps routing/integration
- A ROS IMU publisher for MPU6050
- Hardware-calibrated wheel geometry and encoder constants
- A verified real-world Nav2 tuning pass on the actual wheelchair

## Repository Layout

```text
guido/
|- agents/
|  `- guido_mission_agent/
|- firmware/
|  `- wheelchair_arduino.ino
|- scripts/
|  |- adk_transcript_bridge.py
|  |- install_ros2_humble.sh
|  |- requirements-adk.txt
|  |- setup_lidar.sh
|  |- voice_stream.py
|  `- requirements-voice.txt
`- src/
   |- auto_nav/
   |  |- config/
   |  |  |- nav2.yaml
   |  |  |- slam_nav.rviz
   |  |  `- slam_toolbox.yaml
   |  |- launch/
   |  |  |- navigation.launch.py
   |  |  `- slam_nav.launch.py
   |  |- navigation/
   |  |  |- adapters.py
   |  |  |- command_node.py
   |  |  |- commands.py
   |  |  |- health.py
   |  |  |- messages.py
   |  |  |- navigation_node.py
   |  |  |- supervisor.py
   |  |  |- types.py
   |  |  `- waypoints.py
   |  `- test/
   |- guido_base/
   |  |- config/serial_bridge.yaml
   |  `- guido_base/
   |     |- keyboard_teleop.py
   |     `- serial_bridge.py
   |- guido_bringup/
   |  |- config/
   |  |  |- ldlidar.yaml
   |  |  `- lifecycle_mgr.yaml
   |  |- launch/
   |  |  |- guido_base.launch.py
   |  |  `- guido_lidar.launch.py
   |  `- urdf/guido.urdf.xml
   `- ldrobot-lidar-ros2/   # git submodule, required for LiDAR runtime
```

## Dependencies And Build

### 1. Restore the LiDAR driver submodule

The current checkout has an empty `src/ldrobot-lidar-ros2` gitlink. Restore it before attempting LiDAR bringup:

```bash
git submodule update --init --recursive src/ldrobot-lidar-ros2
```

The LiDAR driver submodule is configured to track the **`humble`** branch (matches ROS 2 Humble on Ubuntu 22.04).

### 2. Install ROS 2 Humble and runtime packages

Use **Ubuntu 22.04 (Jammy)** for official Humble binaries.

```bash
chmod +x scripts/install_ros2_humble.sh
sudo ./scripts/install_ros2_humble.sh
source /opt/ros/humble/setup.bash
```

The install script includes:

- `ros-humble-navigation2`
- `ros-humble-nav2-bringup`
- `ros-humble-nav2-map-server`
- `ros-humble-nav2-lifecycle-manager`
- `ros-humble-slam-toolbox`

### 3. Build the workspace

```bash
chmod +x scripts/setup_lidar.sh
./scripts/setup_lidar.sh
source install/local_setup.bash
```

This does the following:

- installs `python3-serial` and `python3-yaml`
- installs LiDAR udev rules if the submodule is present
- runs `rosdep install --from-paths src --ignore-src -r -y`
- builds the workspace with `colcon`

## How To Activate Each Subsystem

### LiDAR

Expected device:

- `/dev/ldlidar`

Config:

- `src/guido_bringup/config/ldlidar.yaml`

Bringup:

```bash
ros2 launch guido_bringup guido_lidar.launch.py
```

Verify:

```bash
ls -l /dev/ldlidar
ros2 topic echo /ldlidar_node/scan
ros2 node info /ldlidar_node
```

Notes:

- `guido_lidar.launch.py` now launches `ldlidar_node` directly with `ldlidar.yaml`, so the configured serial port, baud rate, and fixed bin count are actually applied.
- If the submodule is still empty, this launch will fail until `src/ldrobot-lidar-ros2` is restored.

### SLAM

Config:

- `src/auto_nav/config/slam_toolbox.yaml`

Bringup:

```bash
ros2 launch auto_nav navigation.launch.py
```

Verify:

```bash
ros2 topic echo /map
ros2 run tf2_tools view_frames
```

Expected TF chain:

- `map -> odom -> base_footprint -> base_link`

### Motors

Expected controller device:

- `src/guido_base/config/serial_bridge.yaml` defaults to:
  - `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`

Bringup:

```bash
ros2 launch guido_bringup guido_base.launch.py
```

Verify:

```bash
ros2 node info /guido_serial_bridge
ros2 topic echo /odom
```

Manual motion test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.03}, angular: {z: 0.0}}" -r 10
```

Stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 10
```

Keyboard teleop:

```bash
ros2 run guido_base keyboard_teleop
```

Keys:

- `w` forward
- `s` reverse
- `a` rotate left
- `d` rotate right
- `space` or `x` stop

### IMU / MPU6050

Current status:

- The MPU6050 is initialized inside the Arduino firmware only
- There is no ROS IMU topic in this repository yet

Code-only verification:

- On Arduino boot, the firmware prints `MPU6050 connection failed` on serial if initialization fails

What still requires hardware work:

- ROS publisher for IMU data
- bias calibration
- covariance tuning

### Voice Commands

Install voice dependencies:

```bash
sudo apt install portaudio19-dev unzip
python3 -m pip install -r scripts/requirements-voice.txt
```

Additional dependency split:

- `scripts/requirements-voice.txt` covers speech-to-text support only
- `scripts/requirements-adk.txt` installs the Google ADK dependency used for waypoint memory and resolution
- `rclpy` comes from the ROS 2 installation and is required for `--ros-command-topic`

Install ADK:

```bash
python3 -m pip install -r scripts/requirements-adk.txt
```

Run offline STT:

```bash
python3 scripts/voice_stream.py --model models/vosk-model-small-en-us-0.15
```

Bridge transcripts to ADK only:

```bash
export GOOGLE_API_KEY='your-key'
python3 scripts/voice_stream.py --model models/vosk-model-small-en-us-0.15 \
  | python3 scripts/adk_transcript_bridge.py
```

Bridge transcripts into ROS `auto_nav` commands:

```bash
python3 scripts/voice_stream.py --model models/vosk-model-small-en-us-0.15 \
  | python3 scripts/adk_transcript_bridge.py --ros-command-topic /auto_nav/command
```

You only need `GOOGLE_API_KEY` and Google ADK installed when you want the bridge to resolve spoken destinations through the mission agent. Direct `cancel_navigation` and `stop` commands can be published into ROS without ADK.

Direct ROS-safe commands supported by the bridge:

- `cancel_navigation`
- `stop`

ADK-resolved waypoint commands published into ROS:

- `save_waypoint <name>`
- `navigate_to <name>`
- `stop`
- `cancel_navigation`

Examples:

- `remember this as charging dock`
- `go back to room 2`
- `take me to charging dock`

The ADK bridge now resolves against the live `auto_nav` waypoint file instead of a hardcoded agent-side list. By default it reads `.guido/waypoints.yaml` under the repository root, and you can override that with `GUIDO_WAYPOINT_FILE=/path/to/waypoints.yaml`.

### Frontend live view

Start the telemetry bridge:

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch guido_bringup guido_frontend_bridge.launch.py
```

Then run the React frontend:

```bash
cd frontend/guido-live-view
npm install
npm run dev
```

The frontend polls `http://localhost:8765/api/state` and `http://localhost:8765/api/map` by default. Override the bridge host with:

```bash
VITE_TELEMETRY_BASE=http://<jetson-ip>:8765 npm run dev
```

## How Robot Motion Works

### Current low-level stack

- `guido_serial_bridge` subscribes to `/cmd_vel`
- It maps differential-drive wheel velocities to PWM
- It writes ASCII commands to the Arduino at `115200`
- The Arduino drives motor direction and PWM pins

### Safety behavior

- Jetson side: `guido_serial_bridge` sends `STOP` when `/cmd_vel` goes silent for `cmd_timeout_sec`
- Arduino side: the firmware now also stops the motors if serial commands go quiet for `500 ms`

### Odometry behavior

- The Arduino integrates encoder ticks and gyro yaw into `ODOM x y theta vLin vAng`
- Encoder direction is not available from the current interrupt wiring
- The firmware now signs wheel travel using the most recent commanded wheel direction
- This is safer than always-positive odometry, but it is still an approximation until true directional encoder decoding is added

### Autonomous motion path

```text
/auto_nav/command
  -> auto_nav_command_node
  -> /auto_nav/request
  -> auto_nav_navigation_node
  -> Nav2 navigate_to_pose
  -> /cmd_vel
  -> guido_serial_bridge
  -> Arduino
  -> motors
```

Waypoint navigation via Nav2 never publishes raw PWM or GPIO commands; it only sends goals that become `/cmd_vel` through Nav2.

## Waypoint navigation

- Save poses with `save_waypoint <name>` on `/auto_nav/command` (requires `map` frame localization).
- Navigate with `navigate_to <name>`.
- List names with `list_waypoints`.
- RViz config for SLAM + map + scan: `src/auto_nav/config/slam_nav.rviz`

## Launch Files

### Base only

```bash
ros2 launch guido_bringup guido_base.launch.py
```

### Base + LiDAR

```bash
ros2 launch guido_bringup guido_lidar.launch.py
```

### Base + LiDAR + SLAM + Nav2 + recovered waypoint navigation

```bash
ros2 launch auto_nav navigation.launch.py
```

Run the launch command from the workspace root unless you override parameters. The recovered launch files write maps and waypoints relative to the current working directory in `.guido/`.

### SLAM + Nav2 + waypoint stack

Bring up SLAM, Nav2, map saver, and `auto_nav` (drive with keyboard teleop or another `/cmd_vel` source to map, then use waypoints):

```bash
ros2 launch auto_nav slam_nav.launch.py
```

Stop the robot / clear Nav2 motion:

```bash
ros2 topic pub /auto_nav/command std_msgs/msg/String "{data: 'stop'}" -1
```

Cancel the current Nav2 goal:

```bash
ros2 topic pub /auto_nav/command std_msgs/msg/String "{data: 'cancel_navigation'}" -1
```

## Calibration And Configuration

### Serial bridge

File:

- `src/guido_base/config/serial_bridge.yaml`

Important parameters:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `serial_port` | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` | Arduino device |
| `baudrate` | `115200` | Must match firmware |
| `wheel_base` | `0.17` | Must match firmware and hardware |
| `max_wheel_vel` | `0.5` | Wheel speed corresponding to PWM 255 |
| `left_min_abs_pwm` | `45` | Helps overcome stiction |
| `right_min_abs_pwm` | `35` | Helps overcome stiction |

### LiDAR

File:

- `src/guido_bringup/config/ldlidar.yaml`

Important parameters:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `comm.serial_port` | `/dev/ldlidar` | LiDAR serial device |
| `comm.baudrate` | `230400` | LD19 baud rate |
| `lidar.model` | `LDLiDAR_LD19` | Driver model |
| `lidar.frame_id` | `ldlidar_link` | Scan frame |
| `lidar.bins` | `455` | Fixed bins for SLAM compatibility |

### SLAM + Nav2

Files:

- `src/auto_nav/config/slam_toolbox.yaml`
- `src/auto_nav/config/nav2.yaml`

Safety-critical Nav2 defaults:

- max linear speed: `0.12 m/s`
- max angular speed: `0.5 rad/s`
- wheelchair footprint: `0.70 m x 0.60 m`
- inflation radius: `0.45-0.50 m`

## Example End-To-End Run Sequence

1. Flash the Arduino with `firmware/wheelchair_arduino.ino`
2. Connect LiDAR, Arduino, and wheelchair power
3. Verify device paths:

```bash
ls -l /dev/ldlidar /dev/serial/by-id/*1a86*
```

4. Source ROS and the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

5. Start SLAM + Nav2 + auto_nav (from the workspace root):

```bash
ros2 launch auto_nav slam_nav.launch.py
```

6. In a second terminal, watch status:

```bash
ros2 topic echo /auto_nav/status
ros2 topic echo /auto_nav/result
```

7. If needed, stop immediately:

```bash
ros2 topic pub /auto_nav/command std_msgs/msg/String "{data: 'stop'}" -1
```

## Safety Checklist

Before any floor test:

- lift the drive wheels or otherwise mechanically isolate the chair
- keep a human at the power disconnect / E-stop
- verify LiDAR scan data is live before enabling Nav2
- verify `/odom` is updating and has the correct sign for forward and reverse
- confirm `map -> odom -> base_footprint` exists
- begin with `0.03-0.05 m/s` manual tests only
- do not run teleop and Nav2 simultaneously
- do not trust wheel geometry or encoder constants until physically calibrated

Stop conditions that should block autonomous motion:

- no LiDAR scan
- stale odom
- stale TF
- no `/map`
- no consumer on `/cmd_vel`

## Troubleshooting

### No `/dev/ldlidar`

```bash
dmesg | grep -i cp210
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Serial bridge cannot open the Arduino port

```bash
groups
sudo usermod -a -G dialout $USER
```

Then log out and back in.

### LiDAR launch fails immediately

Most likely causes:

- the `src/ldrobot-lidar-ros2` submodule is still empty
- `/dev/ldlidar` does not exist
- baud rate does not match the LD19

### Waypoint or Nav2 commands fail

Check:

```bash
ros2 topic echo /auto_nav/status
ros2 topic echo /map
ros2 topic echo /odom
```

Navigation to a waypoint is rejected if scan / odom / TF health is stale. Saving a waypoint requires a valid `map -> base_link` (or configured base frame) transform.

### Waypoint commands fail from voice

Likely causes:

- `GOOGLE_API_KEY` is not set, so ADK cannot resolve destination phrases
- the waypoint has not been saved yet in the active `.guido/waypoints.yaml`
- `--ros-command-topic /auto_nav/command` was not passed

## Remaining Unknowns

These cannot be verified from repository code alone:

- exact wheelchair motor driver wiring
- true wheel radius and track width
- encoder polarity and real ticks-per-revolution
- whether the current PWM scales are safe on the physical chair
- MPU6050 mounting orientation and calibration
- final Nav2 tuning needed for doorways, tight turns, and indoor clutter

Treat all motion-related constants as provisional until validated on the actual wheelchair.
