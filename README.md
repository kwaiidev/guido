# Guido — Autonomous Wheelchair

ROS 2-controlled autonomous wheelchair stack for NVIDIA Jetson, with LiDAR sensing and a serial motor-controller bridge for base motion.

## Hardware

| Component | Model |
|-----------|-------|
| Compute   | NVIDIA Jetson (Orin Nano / AGX Orin) |
| LiDAR     | LDRobot LD19 (360° DToF, 12 m range) |
| Platform  | Power wheelchair with motor controller |

## Project Layout

```
guido/
├── scripts/
│   ├── install_ros2_jazzy.sh   # ROS 2 Jazzy installer (Ubuntu 24.04 / JP6)
│   └── setup_lidar.sh          # workspace deps + LiDAR udev rules + build
├── src/
│   ├── guido_base/             # ROS 2 serial bridge for the wheelchair motor controller
│   │   ├── config/
│   │   │   └── serial_bridge.yaml  # serial port + wheel geometry config
│   │   └── guido_base/
│   │       └── serial_bridge.py    # /cmd_vel -> serial, serial -> /odom + TF
│   ├── guido_bringup/          # Launch files, LiDAR config, wheelchair URDF
│   │   ├── config/
│   │   │   ├── ldlidar.yaml
│   │   │   └── lifecycle_mgr.yaml
│   │   ├── launch/
│   │   │   ├── guido_base.launch.py
│   │   │   └── guido_lidar.launch.py
│   │   └── urdf/
│   │       └── guido.urdf.xml
│   └── ldrobot-lidar-ros2/     # LDLidar ROS 2 driver (jazzy branch)
└── README.md
```

## Setup

### 1. Install ROS 2 Jazzy

```bash
chmod +x scripts/install_ros2_jazzy.sh
sudo ./scripts/install_ros2_jazzy.sh
source ~/.bashrc
```

### 2. Build workspace and configure devices

```bash
chmod +x scripts/setup_lidar.sh
./scripts/setup_lidar.sh
source ~/.bashrc
```

This installs workspace dependencies, adds the current user to `dialout` for serial access, applies the LiDAR udev rule, and builds the ROS workspace.

### 3. Plug in the controller and LiDAR

The wheelchair motor controller should be available at `/dev/ttyUSB1` by default. The LDLidar should appear as `/dev/ldlidar`.

Verify both devices:

```bash
ls -l /dev/ttyUSB1 /dev/ldlidar
```

If the controller port is different on your Jetson, update `src/guido_base/config/serial_bridge.yaml`.

## Usage

### Launch base control only

```bash
ros2 launch guido_bringup guido_base.launch.py
```

This starts:
- **guido_state_publisher** — publishes the wheelchair URDF TF tree
- **guido_serial_bridge** — subscribes to `/cmd_vel`, sends motor commands over serial, and publishes `/odom` plus `odom -> base_footprint`

### Drive the wheelchair through ROS

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}" -r 10
```

Send a zero command to stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 10
```

### Drive with keyboard keybinds

```bash
ros2 launch guido_bringup guido_base.launch.py
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run guido_base keyboard_teleop
```

Keybinds:
- `w` forward
- `s` backward
- `a` left
- `d` right
- `space` or `x` stop

### Launch the full stack

```bash
ros2 launch guido_bringup guido_lidar.launch.py
```

This starts:
- **guido_state_publisher** — publishes the wheelchair URDF TF tree
- **guido_serial_bridge** — exposes wheelchair motion through `/cmd_vel`, `/odom`, and TF
- **ldlidar_node** — LDRobot driver publishing `/ldlidar_node/scan` (`sensor_msgs/LaserScan`)
- **lifecycle_manager** — auto-configures and activates the lidar node

### Verify base topics

```bash
ros2 node info /guido_serial_bridge
ros2 topic echo /odom
```

### Verify scan data

```bash
ros2 topic echo /ldlidar_node/scan
```

### Visualize in RViz2 (desktop only)

```bash
ros2 launch ldlidar_node ldlidar_rviz2.launch.py
```

### Check TF tree

```bash
ros2 run tf2_tools view_frames
```

Expected chain: `base_footprint → base_link → ldlidar_base → ldlidar_link`

## Configuration

Edit `src/guido_base/config/serial_bridge.yaml` to change:

| Parameter | Default | Notes |
|-----------|---------|-------|
| `serial_port` | `/dev/ttyUSB1` | Arduino / motor controller serial device |
| `baudrate` | `115200` | Must match the Arduino firmware |
| `wheel_base` | `0.17` | Meters between wheels |
| `max_wheel_vel` | `0.5` | Wheel speed mapped to PWM 255 |
| `cmd_timeout_sec` | `0.5` | Sends `STOP` if `/cmd_vel` goes silent |

Edit `src/guido_bringup/config/ldlidar.yaml` to change:

| Parameter | Default | Notes |
|-----------|---------|-------|
| `comm.serial_port` | `/dev/ldlidar` | Set by udev rule |
| `lidar.model` | `LDLiDAR_LD19` | Also supports LD06, STL27L |
| `lidar.bins` | `455` | Set to `0` for dynamic sizing |
| `lidar.range_min` | `0.03` | Meters |
| `lidar.range_max` | `15.0` | Meters |
| `lidar.rot_verse` | `CCW` | Use `CW` if mounted upside-down |

## Troubleshooting

**No `/dev/ldlidar` after plugging in:**
```bash
# Check USB detection
dmesg | grep -i cp210
# Manually re-run udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Serial bridge cannot open `/dev/ttyUSB1`:**
```bash
groups
sudo usermod -a -G dialout $USER
# then log out and back in
```

Quick workaround for a single session:
```bash
sudo chmod 666 /dev/ttyUSB1
```

**"Transitioning failed" on lifecycle configure:**
- Verify serial port: `ls -l /dev/ldlidar`
- Check permissions: `sudo chmod 777 /dev/ldlidar`
- Confirm baudrate matches your lidar model (LD19 = 230400)

**Build errors about missing nav2_util:**
```bash
sudo apt install ros-jazzy-nav2-util ros-jazzy-nav2-msgs ros-jazzy-nav2-lifecycle-manager
```

## License

Apache 2.0 — LDLidar driver by [Walter Lucetti / Myzhar](https://github.com/Myzhar/ldrobot-lidar-ros2).
