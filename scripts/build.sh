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

# Ensure Node >= 18 (Express 5+ and other deps require it; Ubuntu/Jetson repos may be too old)
if ! command -v node >/dev/null 2>&1 || ! node -v | grep -qE '^v(18|19|20|21|22)\.'; then
  echo "Node >=18 not found (current: $(node -v 2>/dev/null || echo 'none')). Installing Node 18 (NodeSource)..."
  sudo apt install -y curl ca-certificates gnupg
  curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
fi

sudo apt install -y \
  python3-colcon-common-extensions \
  nodejs npm \
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

echo "[2/4] Installing node deps..."
cd "$WS_DIR/web"
# Prefer package-lock if present; install all deps (ws/express/etc.)
npm install

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
