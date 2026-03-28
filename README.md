# Guido — Autonomous Wheelchair

LiDAR-navigated autonomous wheelchair powered by ROS 2 Jazzy on NVIDIA Jetson.

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
│   └── setup_lidar.sh          # udev rules + rosdep + colcon build
├── src/
│   ├── ldrobot-lidar-ros2/     # LDLidar ROS 2 driver (jazzy branch)
│   └── guido_bringup/          # Guido launch files and config
│       ├── config/
│       │   ├── ldlidar.yaml        # LiDAR parameters
│       │   └── lifecycle_mgr.yaml  # Nav2 lifecycle manager config
│       ├── launch/
│       │   └── guido_lidar.launch.py
│       └── urdf/
│           └── guido.urdf.xml      # Wheelchair URDF
└── README.md
```

## Setup

### 1. Install ROS 2 Jazzy

```bash
chmod +x scripts/install_ros2_jazzy.sh
sudo ./scripts/install_ros2_jazzy.sh
source ~/.bashrc
```

### 2. Build workspace & configure LiDAR

```bash
chmod +x scripts/setup_lidar.sh
./scripts/setup_lidar.sh
source ~/.bashrc
```

### 3. Plug in the LiDAR

Connect the LD19 via USB. Verify the symlink exists:

```bash
ls -l /dev/ldlidar
```

If it doesn't appear, check `dmesg | tail` and verify the CP210x USB-UART bridge is detected.

## Usage

### Launch LiDAR

```bash
ros2 launch guido_bringup guido_lidar.launch.py
```

This starts:
- **guido_state_publisher** — publishes the wheelchair URDF TF tree
- **ldlidar_node** — LDRobot driver publishing `/ldlidar_node/scan` (`sensor_msgs/LaserScan`)
- **lifecycle_manager** — auto-configures and activates the lidar node

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
