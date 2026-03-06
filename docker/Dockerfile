# =============================================================================
# TAMP Dev - Docker Development Environment
# Base: NVIDIA Isaac ROS (ROS2 Humble + CUDA 12.6) on Ubuntu 22.04
# Target: Jetson AGX Orin (arm64) / Development PC (amd64)
# =============================================================================

ARG BASE_IMAGE=nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_f247dd1051869171c3fc53bb35f6b907
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
# Base image has ROS2 Humble installed but ROS_DISTRO env is not set
ENV ROS_DISTRO=humble

# =============================================================================
# Layer 0: Fix base image apt sources (arm64-jetpack)
#
# arm64-jetpack base image의 Isaac ROS apt source에 'external-main'
# 컴포넌트가 포함되어 있으나, noble-jetpack suite에는 존재하지 않아
# "component misspelt in sources.list" 경고 발생. 제거하면 해결.
# amd64에서는 해당 컴포넌트가 존재하므로 제거해도 무해 (main에 동일 패키지 포함).
#
# Jetson repo의 legacy keyring 경고는 기능에 영향 없으므로 무시 가능.
# =============================================================================
RUN sed -i 's/ external-main//' /etc/apt/sources.list.d/nvidia-isaac-ros.list 2>/dev/null || true

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
# Layer 5a: MoveIt + motion planning dependencies
# amd64 base image에는 포함되어 있으나 arm64-jetpack에서는 누락될 수 있음.
# 명시적으로 설치하여 양 아키텍처 모두에서 보장.
# (이미 설치된 경우 apt가 자동 스킵하므로 중복 설치 없음)
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-eigen \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 5b: Common ROS2 packages (universally needed for robotics)
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
#
# IMPORTANT: ros-humble-ur-client-library apt 패키지는 설치 금지!
# apt 버전은 구 API(DashboardResponse 없음)를 사용하여 소스 빌드
# ur_robot_driver v3.7.0과 헤더 충돌 발생.
# rosdep install 시에도 --skip-keys "ur_client_library" 필수.
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
    && apt-get remove -y ros-${ROS_DISTRO}-ur-client-library 2>/dev/null || true \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 7: isaac_ros_cumotion 빌드/런타임 의존성
#
# 전략: 무거운 인프라(curobo CUDA lib, isaac_ros_common, pip-shim 패키지)는
# apt 설치, 실제 플래너 로직(isaac_ros_cumotion 등)은 소스 빌드.
#
# 주요 패키지:
#   ros-humble-isaac-ros-common          - cmake 매크로 + ament resource
#                                         (libnvvpi4 ~318MB 포함, 불가피)
#   ros-humble-curobo-core               - 사전 컴파일 CUDA 확장
#                                         (cupy, torch, warp-lang pip-shim 포함)
#   ros-humble-isaac-ros-cumotion-interfaces - action/srv 메시지 IDL
#   ros-humble-nvblox-msgs               - EsdfAndGradients srv (런타임 dep)
#   ros-humble-isaac-ros-nitros-bridge-ros2  - NITROS bridge
#   ros-humble-isaac-ros-pynitros        - Python NITROS
#   ros-humble-isaac-ros-launch-utils    - launch 헬퍼
#   ros-humble-isaac-manipulator-ros-python-utils - ObjectAttachmentShape 타입
#                                         (cumotion_planner.py 직접 import)
#   ninja-build                         - curobo_core 재빌드 시 필요
#
# 소스 빌드 대상 (cumotion.repos → vcs import 후 colcon build):
#   isaac_ros_cumotion, isaac_ros_cumotion_moveit,
#   isaac_ros_cumotion_robot_description, isaac_ros_cumotion_python_utils,
#   isaac_ros_cumotion_examples, ur_moveit_config
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-isaac-ros-common \
    ros-${ROS_DISTRO}-curobo-core \
    ros-${ROS_DISTRO}-isaac-ros-cumotion-interfaces \
    ros-${ROS_DISTRO}-nvblox-msgs \
    ros-${ROS_DISTRO}-isaac-ros-nitros-bridge-ros2 \
    ros-${ROS_DISTRO}-isaac-ros-pynitros \
    ros-${ROS_DISTRO}-isaac-ros-launch-utils \
    ros-${ROS_DISTRO}-isaac-manipulator-ros-python-utils \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Layer 7.5: rosdep 캐시 (컨테이너 시작 시 재실행 방지)
# =============================================================================
RUN rosdep init 2>/dev/null || true && \
    rosdep update --rosdistro=${ROS_DISTRO} 2>/dev/null || true

# =============================================================================
# Layer 8: Workspace setup
# =============================================================================
RUN mkdir -p /workspaces/tamp_ws/src

WORKDIR /workspaces/tamp_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
