#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("livestreaming_cpp")
    web_dir = os.path.join(pkg_share, "web")
    server_js = os.path.join(web_dir, "server.js")
    http_server_js = os.path.join(web_dir, "http_server.js")

    image_topic = LaunchConfiguration("image_topic")
    ws_url = LaunchConfiguration("ws_url")
    http_port = LaunchConfiguration("http_port")
    fps = LaunchConfiguration("fps")
    bitrate_kbps = LaunchConfiguration("bitrate_kbps")

    # Start signaling server first (WS:9002)
    signaling = ExecuteProcess(
        cmd=["node", server_js],
        cwd=web_dir,
        output="screen"
    )

    # Start HTTP server (Express) instead of python3 http.server
    # It serves index.html publicly and protects admin.html with Basic Auth.
    http = ExecuteProcess(
        cmd=["node", http_server_js],
        cwd=web_dir,
        output="screen"
    )

    # Start ROS2 streamer AFTER a short delay so server.js is listening
    streamer = Node(
        package="livestreaming_cpp",
        executable="server",
        name="webrtc_streamer",
        output="screen",
        parameters=[{
            "image_topic": image_topic,
            "ws_url": ws_url,
            "fps": fps,
            "bitrate_kbps": bitrate_kbps,
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        # UPDATED DEFAULT TOPIC HERE:
        DeclareLaunchArgument("image_topic", default_value="/zed/zed_node/rgb/color/rect/image"),
        DeclareLaunchArgument("ws_url", default_value="ws://127.0.0.1:9002"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("bitrate_kbps", default_value="2000"),

        signaling,
        http,

        TimerAction(period=1.0, actions=[streamer]),
    ])
