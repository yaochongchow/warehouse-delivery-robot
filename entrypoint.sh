#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Ensure Gazebo can find ROS2 plugins
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}

# Start virtual framebuffer
export DISPLAY=:99
Xvfb :99 -screen 0 1280x720x24 &
sleep 1

# Start VNC server on the virtual display
x11vnc -display :99 -forever -nopw -shared -rfbport 5900 &

# Start noVNC web client (browser access on port 6080)
/usr/share/novnc/utils/launch.sh --vnc localhost:5900 --listen 6080 &

exec "$@"
