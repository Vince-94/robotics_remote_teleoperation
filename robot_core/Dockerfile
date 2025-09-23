# robot_core/Dockerfile
# Minimal ROS2 image that builds a colcon workspace and runs the robot entrypoint.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    WORKSPACE=/root/ros_ws

# Install build tools and Python deps
RUN apt update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    ca-certificates \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Install any Python tools you want in the container (extend as needed)
RUN apt update && apt-get install -y python3-empy python3-lark

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libopencv-dev \
    python3-opencv \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

# Copy workspace source (your ROS packages) into the image
# Put your ROS2 packages under robot_core/src/ in the repo.
COPY ./src ./src

# (Optional) If you have a package.xml/rosdep requirements, install them here.
# If you keep rosdep YAMLs, you can run rosdep; it may require network.
# RUN apt-get update && apt-get install -y python3-rosdep \
#  && rosdep update \
#  && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

# Copy entrypoint script that sources workspace and launches the demo
COPY ./entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Expose any ports you might use (for rosbridge, RTSP, etc.)
EXPOSE 11311 8080

# Default entrypoint: source setup, run the entrypoint script
ENTRYPOINT ["/ros_entrypoint.sh"]
# Example CMD inside entrypoint can launch a ros2 launch file or drop to a shell
# You can override this from docker-compose with command:
# command: ["ros2", "launch", "robot_launch teleop.launch.py"]
