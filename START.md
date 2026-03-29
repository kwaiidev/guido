# Guido: Quick Start

## Build

```bash
cd ~/guido
colcon build --symlink-install --packages-select auto_nav guido_base guido_bringup
source install/setup.bash
```

## SLAM + Keyboard Teleop (manual mapping)

```bash
# Terminal 1
source ~/guido/install/setup.bash
ros2 launch auto_nav slam_teleop.launch.py

# Terminal 2
source ~/guido/install/setup.bash
ros2 run guido_base keyboard_teleop
```

## Frontier Mapping (autonomous exploration)

```bash
# Terminal 1: SLAM + Nav2 infrastructure
source ~/guido/install/setup.bash
ros2 launch auto_nav frontier_mapping.launch.py

# Terminal 2: interactive frontier explorer
source ~/guido/install/setup.bash
ros2 run auto_nav frontier_explorer
```

In Terminal 2 you approve each frontier before the robot moves:
- **y** -- approve and navigate to the proposed frontier
- **n** -- skip and blacklist that frontier
- **c** -- cancel the current navigation
- **s** -- save map and quit

Optional third terminal (manual nudge while frontier runs): publishes to the same `/cmd_vel` as Nav2; press **c** in the explorer first to avoid fighting the controller.

Map is saved to `~/guido/maps/frontier_map.pgm` / `.yaml`.
