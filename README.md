# livestreaming_cpp

ROS 2 + GStreamer WebRTC livestreaming for ZED camera with **H264 video**, **DataChannel joystick control**, **cmd_vel_out publish**, and **zed_heartbeat** monitoring.

This project runs three parts together using a ROS 2 launch file:
1) **Signaling server (Node.js WebSocket)** – routes offer/answer/ICE between browser and robot  
2) **Static web server** – serves `index.html` and `admin.html`  
3) **ROS 2 C++ streamer node** – subscribes to ZED image topic, encodes H264 via GStreamer, streams via WebRTC, receives joystick values via DataChannel and publishes `/cmd_vel_out`

---

## Features

- ✅ Live **H264 WebRTC video** from `/zed/zed_node/rgb/image_rect_color`
- ✅ **Low-latency** pipeline tuned for realtime streaming (frame drops are allowed vs buffering)
- ✅ DataChannel joystick → publishes `cmd_vel_out`
- ✅ `zed_heartbeat` topic:
  - publishes `1` when frames are arriving
  - publishes `0` when ZED topic stops / stalls
- ✅ **Admin page** (password-protected) that bypasses normal user video-slot limits
- ✅ Normal user: video only if a slot is free, otherwise control-only (handled by signaling layer)

---

## Requirements

### ROS 2
- ROS 2 Humble

### System packages
```bash
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
  libgstreamer-plugins-bad1.0-dev \
  libssl-dev
