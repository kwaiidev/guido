#!/usr/bin/env bash
set -euo pipefail

#  Post-ROS2-install setup: device rules, rosdep, colcon build
#  Run from the workspace root: ./scripts/setup_lidar.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ROS 2 environment not sourced. Sourcing Jazzy..."
  source /opt/ros/jazzy/setup.bash
fi

echo "============================================"
echo " Guido workspace setup  (workspace: $WS_DIR)"
echo "============================================"

echo ""
echo "[1/4] Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y \
  libudev-dev \
  python3-serial

if ! id -nG "$USER" | grep -qw dialout; then
  sudo usermod -a -G dialout "$USER"
  echo "  -> Added $USER to dialout. Log out and back in for serial access."
fi

echo ""
echo "[2/4] Installing udev rules for LDLidar..."
RULES_SRC="$WS_DIR/src/ldrobot-lidar-ros2/rules/ldlidar.rules"
if [ -f "$RULES_SRC" ]; then
  sudo cp "$RULES_SRC" /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  echo "  -> /dev/ldlidar symlink will appear when lidar is plugged in"
else
  echo "  WARNING: $RULES_SRC not found, skipping udev rules"
fi

echo ""
echo "[3/4] Installing ROS dependencies with rosdep..."
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y

echo ""
echo "[4/4] Building workspace..."
cd "$WS_DIR"
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

echo ""
SETUP_FILE="$WS_DIR/install/local_setup.bash"
BASHRC="$HOME/.bashrc"
if ! grep -q "$SETUP_FILE" "$BASHRC" 2>/dev/null; then
  echo "source $SETUP_FILE" >> "$BASHRC"
  echo "  -> Added workspace overlay to ~/.bashrc"
fi

echo ""
echo "============================================"
echo " Build complete!"
echo ""
echo " Source the workspace:"
echo "   source $SETUP_FILE"
echo ""
echo " Launch base control:"
echo "   ros2 launch guido_bringup guido_base.launch.py"
echo ""
echo " Launch the full stack:"
echo "   ros2 launch guido_bringup guido_lidar.launch.py"
echo "============================================"
