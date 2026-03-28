# Guido - Autonomous Wheelchair

ROS 2-controlled autonomous wheelchair stack for NVIDIA Jetson, with LiDAR sensing and a serial motor-controller bridge for base motion.

## Hardware

| Component | Model |
|-----------|-------|
| Compute   | NVIDIA Jetson (Orin Nano / AGX Orin) |
| LiDAR     | LDRobot LD19 (360 deg DToF, 12 m range) |
| Platform  | Power wheelchair with motor controller |

## Project Layout

```text
guido/
|- scripts/
|  |- install_ros2_jazzy.sh   # ROS 2 Jazzy installer (Ubuntu 24.04 / JP6)
|  |- setup_lidar.sh          # workspace deps + LiDAR udev rules + build
|  |- voice_stream.py         # microphone -> stdout speech transcription
|  |- requirements-voice.txt  # Python deps for voice_stream.py
|  `- guido_git_steward.sh    # helper for repo maintenance
|- agents/
|  `- guido_mission_agent/    # existing minimal ADK app entrypoint
|- src/
|  |- guido_base/             # ROS 2 serial bridge for the wheelchair motor controller
|  |  |- config/
|  |  |  `- serial_bridge.yaml
|  |  `- guido_base/
|  |     |- serial_bridge.py  # /cmd_vel -> serial, serial -> /odom + TF
|  |     `- keyboard_teleop.py
|  |- guido_bringup/          # launch files, LiDAR config, wheelchair URDF
|  |  |- config/
|  |  |  |- ldlidar.yaml
|  |  |  `- lifecycle_mgr.yaml
|  |  |- launch/
|  |  |  |- guido_base.launch.py
|  |  |  `- guido_lidar.launch.py
|  |  `- urdf/
|  |     `- guido.urdf.xml
|  `- ldrobot-lidar-ros2/     # LDLidar ROS 2 driver (jazzy branch)
`- README.md
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
- **guido_state_publisher** - publishes the wheelchair URDF TF tree
- **guido_serial_bridge** - subscribes to `/cmd_vel`, sends motor commands over serial, and publishes `/odom` plus `odom -> base_footprint`

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
- **guido_state_publisher** - publishes the wheelchair URDF TF tree
- **guido_serial_bridge** - exposes wheelchair motion through `/cmd_vel`, `/odom`, and TF
- **ldlidar_node** - LDRobot driver publishing `/ldlidar_node/scan` (`sensor_msgs/LaserScan`)
- **lifecycle_manager** - auto-configures and activates the lidar node

### Verify base topics

```bash
ros2 node info /guido_serial_bridge
ros2 topic echo /odom
```

### Existing ADK mission agent

The repo already contains a minimal ADK mission agent under `agents/`, but it remains separate from the ROS stack and from any voice input utility.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install google-adk
export GOOGLE_API_KEY='your-key'
export GUIDO_ADK_MODEL=gemini-2.5-flash
cd agents
adk web
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

Expected chain: `base_footprint -> base_link -> ldlidar_base -> ldlidar_link`

## Voice Transcription Utility

`scripts/voice_stream.py` is a standalone terminal transcription utility. It captures microphone audio from the local machine, performs continuous speech-to-text with Vosk, writes transcript lines to stdout, and keeps logs on stderr so it can be piped into another process later.

### Install voice dependencies

Install the audio runtime and Python packages:

```bash
sudo apt install portaudio19-dev unzip
python3 -m pip install -r scripts/requirements-voice.txt
```

For a fully offline local model, download and unpack a Vosk model. Example for the small US English model:

```bash
mkdir -p models
curl -L -o /tmp/vosk-model-small-en-us-0.15.zip \
  https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip -q /tmp/vosk-model-small-en-us-0.15.zip -d models
```

This creates a model directory like `models/vosk-model-small-en-us-0.15`.

### Run the voice stream

Plain text, one finalized utterance per line:

```bash
python3 scripts/voice_stream.py --model models/vosk-model-small-en-us-0.15
```

List microphone devices first if needed:

```bash
python3 scripts/voice_stream.py --list-devices
```

Select a specific device or sample rate:

```bash
python3 scripts/voice_stream.py \
  --model models/vosk-model-small-en-us-0.15 \
  --device 2 \
  --samplerate 16000
```

Emit partial hypotheses too:

```bash
python3 scripts/voice_stream.py \
  --model models/vosk-model-small-en-us-0.15 \
  --partials
```

Emit JSONL for downstream consumers:

```bash
python3 scripts/voice_stream.py \
  --model models/vosk-model-small-en-us-0.15 \
  --jsonl \
  --timestamps
```

Pipe transcript output into another process:

```bash
python3 scripts/voice_stream.py --model models/vosk-model-small-en-us-0.15 | some_other_process
```

If you omit `--model`, the script can try `--language en-us`, but passing `--model` is the recommended path for predictable offline operation.

### Linux microphone notes

- Use `--list-devices` if the default input is not the microphone you want.
- On Jetson or headless Ubuntu systems, a USB microphone is usually the simplest option.
- If the stream cannot open, verify the device appears in ALSA or PulseAudio and that the requested sample rate is supported.
- Startup logs and audio warnings go to stderr by design, so stdout stays pipe-friendly.

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

## Minimal ADK Tools

The current ADK app exposes only five simple tools:

- `list_destinations`
- `lookup_destination`
- `get_robot_status`
- `send_mission`
- `cancel_mission`

This is enough to demo command understanding and mission packaging on the Nano. It does not drive the chair and it does not replace ROS safety logic.

## License

Apache 2.0 - LDLidar driver by [Walter Lucetti / Myzhar](https://github.com/Myzhar/ldrobot-lidar-ros2).
