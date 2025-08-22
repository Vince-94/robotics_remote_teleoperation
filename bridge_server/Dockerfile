# bridge_server/Dockerfile
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    APP_HOME=/bridge

WORKDIR ${APP_HOME}

# install python deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    git \
 && rm -rf /var/lib/apt/lists/*

# rclpy is available in ROS base; install Python-side requirements
RUN pip3 install --no-cache-dir websockets aiohttp

# copy the bridge code
COPY ./bridge.py ./bridge.py
COPY ./config ./config

EXPOSE 9000

# Run the bridge script on container start
CMD ["bash", "-lc", "source /opt/ros/${ROS_DISTRO}/setup.bash && python3 /bridge/bridge.py"]
