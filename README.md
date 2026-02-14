# Livestreaming_CPP (ROS 2 + WebRTC) — Nitros Enabled

This package streams a ROS 2 camera topic to a browser using **GStreamer WebRTC** and supports **NVIDIA Isaac ROS Nitros** (for high-performance image transport).  
It also includes a web UI (normal + admin page) served by a Node.js HTTP server and a websocket signaling server.

---

## What this uses

- **ROS 2 Humble**
- **Isaac ROS Nitros** (Nitros image subscription / acceleration)
- **GStreamer WebRTC** (`webrtcbin`, H264 encoding, RTP payloading)
- **Node.js servers**
  - HTTP UI: `http://<IP>:8080/`
  - Admin UI: `http://<IP>:8080/admin`
  - Signaling WebSocket: `ws://<IP>:9002`

---

## Folder Layout

- Workspace root: `~/Livestreaming_CPP`
- Build script: `~/Livestreaming_CPP/scripts/build.sh`

---

## Prerequisites

### 1) ROS 2 Humble

Make sure ROS 2 Humble is installed and source it:

```bash
source /opt/ros/humble/setup.bash
```

# Build & Install (using build.sh)

The build script is located here: `/scripts/build.sh`

---

## 1) Make it executable

From the workspace root folder:

```bash
cd ~/Music/Livestreaming_CPP
chmod +x scripts/build.sh
```
## 2) Run the build script

```bash
./scripts/build.sh
```

This script will:

install required apt dependencies (GStreamer plugins, dev libs, etc.)

install Node dependencies for the web server

build the ROS 2 package using colcon

## 3) Launch the Streamer

```bash
ros2 launch livestreaming_cpp stream.launch.py
```