FROM osrf/ros:humble-desktop-full

# Stage 1: Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-plugins \
    ros-humble-rosbridge-server \
    ros-humble-tf2-web-republisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    python3-pip \
    xvfb x11vnc novnc websockify \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install rich textual

ENV ROS_DOMAIN_ID=0
ENV GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
WORKDIR /ros2_ws

# Stage 2: Copy source and build
COPY src/ /ros2_ws/src/
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null; \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
