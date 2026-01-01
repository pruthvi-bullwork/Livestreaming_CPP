#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[1/4] Installing apt dependencies..."
sudo apt update
sudo apt install -y \
  nodejs npm \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev

echo "[2/4] Installing node deps..."
cd "$WS_DIR/web"
npm install ws

echo "[3/4] Building ROS2 package..."
cd "$WS_DIR"
rm -rf build install log
colcon build --packages-select livestreaming_cpp

echo "[4/4] Done."
echo "Now run:"
echo "  source install/setup.bash"
echo "  ros2 launch livestreaming_cpp stream.launch.py"
