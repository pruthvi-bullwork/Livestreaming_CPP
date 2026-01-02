#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[0/4] Sourcing ROS 2 Humble..."
if [ -f /opt/ros/humble/setup.bash ]; then
  # ROS setup scripts can reference unset vars; keep bash non-strict here
  set +u
  source /opt/ros/humble/setup.bash
  set -u 2>/dev/null || true
else
  echo "ERROR: /opt/ros/humble/setup.bash not found. Install ROS2 Humble first."
  exit 1
fi

echo "[1/4] Installing apt dependencies..."
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
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

echo "[2/4] Installing node deps (only ws)..."
cd "$WS_DIR/web"
npm install ws

echo "[3/4] Building ROS2 package..."
cd "$WS_DIR"

# If old build artifacts are owned by root (happens if colcon was run with sudo),
# fix ownership so we can clean normally.
if [ -d build ] || [ -d install ] || [ -d log ]; then
  if [ ! -w build ] 2>/dev/null || [ ! -w install ] 2>/dev/null || [ ! -w log ] 2>/dev/null; then
    echo "Build folders not writable (likely root-owned). Fixing ownership..."
    sudo chown -R "$USER:$USER" build install log || true
  fi
fi

rm -rf build install log
colcon build --packages-select livestreaming_cpp


echo "[4/4] Done."
echo "Run:"
echo "  source install/setup.bash"
echo "  ros2 launch livestreaming_cpp stream.launch.py"
