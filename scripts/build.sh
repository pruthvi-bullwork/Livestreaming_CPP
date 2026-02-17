#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[0/4] Cleaning up broken NVIDIA repositories..."
# Remove 404 repositories identified in your logs
sudo rm -f /etc/apt/sources.list.d/isaac-ros-embeddedsw.list || true
sudo rm -f /etc/apt/sources.list.d/isaac-ros.list || true

echo "[1/4] Sourcing ROS 2 Humble..."
if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u 2>/dev/null || true
else
  echo "ERROR: /opt/ros/humble/setup.bash not found. Install ROS2 Humble first."
  exit 1
fi

echo "[2/4] Installing apt dependencies..."
sudo apt update || true

# Modern NodeSource installs nodejs which includes npm; avoid installing npm separately
sudo apt install -y \
  python3-colcon-common-extensions \
  nodejs \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-nice \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev

echo "[3/4] Installing node deps..."
cd "$WS_DIR/web"
# Use 'npm install' to ensure all dependencies from package.json are satisfied
npm install

echo "[4/4] Building ROS2 package..."
cd "$WS_DIR"

# Standard permission fix for Jetson workspaces
if [ -d build ] || [ -d install ] || [ -d log ]; then
  sudo chown -R "$USER:$USER" build install log || true
fi

rm -rf build install log
colcon build --packages-select livestreaming_cpp

echo "Done. Run: source install/setup.bash && ros2 launch livestreaming_cpp stream.launch.py"