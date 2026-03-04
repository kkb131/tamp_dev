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

# Generate ur10e URDF for cuMotion if not already present
URDF_TARGET=/workspaces/tamp_ws/ur10e.urdf
URDF_CACHED=/workspaces/tamp_ws/src/tamp_dev/.docker/assets/ur10e.urdf
if [ ! -f "${URDF_TARGET}" ]; then
    if [ -f "${URDF_CACHED}" ]; then
        cp "${URDF_CACHED}" "${URDF_TARGET}"
        echo "[entrypoint] Restored ur10e.urdf from cache."
    elif command -v xacro &>/dev/null && ros2 pkg prefix ur_description &>/dev/null 2>&1; then
        XACRO=$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro
        xacro "${XACRO}" ur_type:=ur10e name:=ur > "${URDF_TARGET}" 2>/dev/null && \
            cp "${URDF_TARGET}" "${URDF_CACHED}" && \
            echo "[entrypoint] Generated ur10e.urdf."
    fi
fi

# Initialize rosdep if not already done (cache is built into image)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init 2>/dev/null || true
    rosdep update --rosdistro="${ROS_DISTRO}" 2>/dev/null || true
fi

# Clean up stale manual stub (apt version supersedes)
STALE_STUB="/opt/ros/jazzy/share/isaac_ros_common/scripts/isaac_ros_common-version-info.py"
if [ -f "${STALE_STUB}" ] && ! dpkg -S "${STALE_STUB}" >/dev/null 2>&1; then
    rm -f "${STALE_STUB}"
    rm -rf "$(dirname "${STALE_STUB}")/__pycache__"
fi

# Execute the command passed to the container
exec "$@"
