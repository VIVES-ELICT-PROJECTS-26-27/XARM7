# syntax=docker/dockerfile:1
FROM tiryoh/ros2-desktop-vnc:humble

# Step 1: Install core tools + Stability components (CycloneDDS)
RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Step 2: Build the xArm workspace INTO the image
WORKDIR /home/ubuntu/ros2_ws
RUN mkdir src && cd src && \
    git clone --depth 1 https://github.com/xArm-Developer/xarm_ros2.git --recursive

# Install dependencies and build
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble || true && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# keypad_ctrl.py streams Cartesian velocity through MoveIt Servo.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit-servo \
    && rm -rf /var/lib/apt/lists/*

# moveit2_tutorials is built in the overlay. These libraries must live in
# the image: apt installs inside a container disappear with --rm.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit-ros-perception \
    ros-humble-moveit-visual-tools \
    ros-humble-rviz-visual-tools \
    ros-humble-moveit-task-constructor-core \
    ros-humble-moveit-hybrid-planning \
    ros-humble-moveit-resources-panda-moveit-config \
    ros-humble-moveit-configs-utils \
    ros-humble-gripper-controllers \
    && rm -rf /var/lib/apt/lists/*

# Step 3: Copy config files (changes here trigger fast rebuild)
COPY cyclonedds.xml /cyclonedds.xml
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]