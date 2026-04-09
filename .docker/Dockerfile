FROM osrf/ros:humble-desktop-full

# Stage 1: Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-rosbridge-server \
    ros-humble-teleop-twist-keyboard \
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install rich textual

ENV ROS_DOMAIN_ID=0
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
