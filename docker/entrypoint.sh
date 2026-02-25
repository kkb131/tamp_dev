#!/bin/bash
set -e

# Source ROS2 installation
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source workspace overlay if it has been built
if [ -f /workspaces/tamp_ws/install/setup.bash ]; then
    source /workspaces/tamp_ws/install/setup.bash
    echo "[entrypoint] Sourced workspace overlay."
fi

# Initialize rosdep if not already done
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init 2>/dev/null || true
fi
rosdep update --rosdistro="${ROS_DISTRO}" 2>/dev/null || true

# Execute the command passed to the container
exec "$@"
