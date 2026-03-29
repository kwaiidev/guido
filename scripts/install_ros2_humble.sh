#!/usr/bin/env bash
set -euo pipefail

#  ROS 2 Humble Hawksbill installer for Ubuntu 22.04 (Jammy)
#  Run: chmod +x scripts/install_ros2_humble.sh && sudo ./scripts/install_ros2_humble.sh

echo "=========================================="
echo " Installing ROS 2 Humble (Ubuntu 22.04 Jammy)"
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

echo "Installing ROS 2 Humble desktop + dev tools..."
apt-get install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi
rosdep update --rosdistro humble

echo "Installing LDLidar / Nav2 dependencies..."
apt-get install -y \
  libudev-dev \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-map-server \
  ros-humble-nav2-util \
  ros-humble-nav2-msgs \
  ros-humble-robot-state-publisher \
  ros-humble-diagnostic-updater \
  ros-humble-bond \
  ros-humble-bondcpp \
  ros-humble-slam-toolbox

BASHRC="/home/${SUDO_USER:-$USER}/.bashrc"
if ! grep -q "ros/humble/setup.bash" "$BASHRC" 2>/dev/null; then
  echo "" >> "$BASHRC"
  echo "# ROS 2 Humble" >> "$BASHRC"
  echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
fi

echo ""
echo "=========================================="
echo " ROS 2 Humble installation complete!"
echo " Open a new terminal or run:"
echo "   source /opt/ros/humble/setup.bash"
echo "=========================================="
