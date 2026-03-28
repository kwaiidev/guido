#!/usr/bin/env bash
set -euo pipefail

#  ROS 2 Jazzy Jalisco installer for Ubuntu 24.04 (Desktop & Jetson JP6)
#  Run: chmod +x scripts/install_ros2_jazzy.sh && sudo ./scripts/install_ros2_jazzy.sh

echo "=========================================="
echo " Installing ROS 2 Jazzy (Ubuntu 24.04)"
echo "=========================================="

apt-get update && apt-get install -y \
  software-properties-common \
  curl \
  gnupg \
  lsb-release

add-apt-repository universe -y

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt-get update

echo "Installing ROS 2 Jazzy desktop + dev tools..."
apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi
rosdep update --rosdistro jazzy

echo "Installing LDLidar dependencies..."
apt-get install -y \
  libudev-dev \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-util \
  ros-jazzy-nav2-msgs \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-bond \
  ros-jazzy-bondcpp \
  ros-jazzy-slam-toolbox

BASHRC="/home/${SUDO_USER:-$USER}/.bashrc"
if ! grep -q "ros/jazzy/setup.bash" "$BASHRC" 2>/dev/null; then
  echo "" >> "$BASHRC"
  echo "# ROS 2 Jazzy" >> "$BASHRC"
  echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC"
fi

echo ""
echo "=========================================="
echo " ROS 2 Jazzy installation complete!"
echo " Open a new terminal or run:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "=========================================="
