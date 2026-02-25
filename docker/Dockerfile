# =============================================================================
# TAMP Dev - Docker Development Environment
# Base: NVIDIA Isaac ROS (ROS2 Jazzy + CUDA) on Ubuntu 24.04
# Target: Jetson AGX Orin (arm64) / Development PC (amd64)
# =============================================================================

ARG BASE_IMAGE=nvcr.io/nvidia/isaac/ros:isaac_ros_740c8500df2685ab1f4a4e53852601df-amd64
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
# Base image has ROS2 Jazzy installed but ROS_DISTRO env is not set
ENV ROS_DISTRO=jazzy

# =============================================================================
# Layer 1: System-level development tools
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    nano \
    htop \
    tmux \
    gdb \
    less \
    tree \
    unzip \
    bash-completion \
    software-properties-common \
    usbutils \
    udev \
    iproute2 \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 2: ROS2 GUI packages (RViz2, rqt)
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-rqt-topic \
    ros-${ROS_DISTRO}-rqt-reconfigure \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 3: Python development tools
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-vcstool \
    python3-pytest \
    python3-pytest-cov \
    python3-flake8 \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 4: CUDA toolkit (nvcc for building GPU-accelerated packages)
# Needed later for: cuMotion, FoundationPose, etc.
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-toolkit \
    libnvinfer-dev \
    libnvinfer-plugin-dev \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 5: Common ROS2 packages (universally needed for robotics)
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 6: UR Robot Driver binary dependencies
# ros2_control 프레임워크 및 UR 관련 빌드 의존성만 설치.
# UR 패키지 자체(ur_description, ur_robot_driver, ur_client_library)는
# 소스 빌드이므로 apt 설치 제외.
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ur-msgs \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-generate-parameter-library \
    libeigen3-dev \
    socat \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 7: Workspace setup
# =============================================================================
RUN mkdir -p /workspaces/tamp_ws/src

WORKDIR /workspaces/tamp_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
