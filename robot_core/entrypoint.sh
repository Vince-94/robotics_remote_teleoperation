#!/bin/bash
set -e

WORKSPACE=/root/ros_ws

export force_color_prompt=yes
source ~/.bashrc

# Source ROS and workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ ! -f "${WORKSPACE}/install/setup.bash" ]; then
    echo "[entrypoint] workspace not built, running colcon build (this may take a while)..."
    cd ${WORKSPACE}
    # ensure dependencies - optional, can be slow
    # rosdep update || true
    # rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} || true
    colcon build --symlink-install || { echo "colcon build failed"; exit 1; }
fi

# Source workspace if available
if [ -f "${WORKSPACE}/install/local_setup.bash" ]; then
    source ${WORKSPACE}/install/local_setup.bash
fi

# If user provided a CMD override, exec it; otherwise launch default demo
if [ "$#" -gt 0 ]; then
    exec "$@"
fi
