# =============================================================================
# TAMP Dev - Docker Development Environment
# Base: NVIDIA Isaac ROS (ROS2 Jazzy + CUDA) on Ubuntu 24.04
# Target: Jetson AGX Orin (arm64) / Development PC (amd64)
# =============================================================================

ARG BASE_IMAGE=nvcr.io/nvidia/isaac/ros:isaac_ros_740c8500df2685ab1f4a4e53852601df-amd64
FROM ${BASE_IMAGE}

# Base image pre-installed packages (do NOT re-install):
#   ros-jazzy-moveit (full stack: core, servo, planning, visualization)
#   ros-jazzy-pinocchio (rigid body dynamics, used by servo scripts)
#   ros-jazzy-joy (joystick driver)
#   ros-jazzy-tf2-ros, ros-jazzy-tf2-eigen (transforms)
#   Total: ~425 ros-jazzy-* packages

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
#
# IMPORTANT: ros-jazzy-ur-client-library apt 패키지는 설치 금지!
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
#   ros-jazzy-isaac-ros-common          - cmake 매크로 + ament resource
#                                         (libnvvpi4 ~318MB 포함, 불가피)
#   ros-jazzy-curobo-core               - 사전 컴파일 CUDA 확장
#                                         (cupy, torch, warp-lang pip-shim 포함)
#   ros-jazzy-isaac-ros-cumotion-interfaces - action/srv 메시지 IDL
#   ros-jazzy-nvblox-msgs               - EsdfAndGradients srv (런타임 dep)
#   ros-jazzy-isaac-ros-nitros-bridge-ros2  - NITROS bridge
#   ros-jazzy-isaac-ros-pynitros        - Python NITROS
#   ros-jazzy-isaac-ros-launch-utils    - launch 헬퍼
#   ros-jazzy-isaac-manipulator-ros-python-utils - ObjectAttachmentShape 타입
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
