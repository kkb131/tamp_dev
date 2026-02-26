#!/bin/bash
# =============================================================================
# run_container.sh - Create and run the tamp_dev Docker container
#
# Usage:
#   ./run_container.sh                    # Start interactive session
#   ./run_container.sh --name my_session  # Custom container name
#   ./run_container.sh --join             # Join existing container
#   ./run_container.sh --no-devices       # Skip robot device mounting
#   ./run_container.sh ros2 launch ...    # Run a specific command
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# --- Default configuration ---
IMAGE_NAME="tamp_dev"
IMAGE_TAG="latest"
CONTAINER_NAME="tamp_dev"
WORKSPACE_DIR="${PROJECT_DIR}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
MOUNT_DEVICES=true

# Load overrides from .env.docker if present
ENV_FILE="${PROJECT_DIR}/.env.docker"
if [ -f "${ENV_FILE}" ]; then
    # shellcheck source=/dev/null
    source "${ENV_FILE}"
fi

# --- Argument parsing ---
JOIN_EXISTING=false
USER_CMD=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --name)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --join)
            JOIN_EXISTING=true
            shift
            ;;
        --image)
            IMAGE_NAME="$2"
            shift 2
            ;;
        --tag)
            IMAGE_TAG="$2"
            shift 2
            ;;
        --no-devices)
            MOUNT_DEVICES=false
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS] [COMMAND]"
            echo ""
            echo "Options:"
            echo "  --name NAME       Container name (default: tamp_dev)"
            echo "  --join            Join an existing running container"
            echo "  --image IMAGE     Image name (default: tamp_dev)"
            echo "  --tag TAG         Image tag (default: latest)"
            echo "  --no-devices      Skip robot device auto-mounting"
            echo "  -h, --help        Show this help"
            echo ""
            echo "Examples:"
            echo "  $0                          # Interactive bash session"
            echo "  $0 --join                   # Open new terminal in running container"
            echo "  $0 ros2 launch my_pkg ...   # Run a specific ROS2 command"
            exit 0
            ;;
        --)
            shift
            USER_CMD="$*"
            break
            ;;
        *)
            USER_CMD="$*"
            break
            ;;
    esac
done

# --- Join existing container ---
if [ "${JOIN_EXISTING}" = true ]; then
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "[run] Joining existing container: ${CONTAINER_NAME}"
        docker exec -it "${CONTAINER_NAME}" bash
        exit 0
    else
        echo "ERROR: Container '${CONTAINER_NAME}' is not running."
        echo "       Running containers:"
        docker ps --format '  {{.Names}} ({{.Image}})' 2>/dev/null || echo "  (none)"
        exit 1
    fi
fi

# --- Check if container already exists ---
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "[run] Container '${CONTAINER_NAME}' is already running."
        echo "      Use --join to attach, or stop it first:"
        echo "      docker stop ${CONTAINER_NAME}"
        exit 1
    else
        echo "[run] Removing stopped container: ${CONTAINER_NAME}"
        docker rm "${CONTAINER_NAME}"
    fi
fi

# --- GPU passthrough ---
# 감지 우선순위:
#   1. /dev/nvidia0 존재 여부 (가장 신뢰할 수 있는 지표)
#   2. docker info에서 nvidia runtime 확인
#   3. nvidia-smi 경로 확인 (PATH 이외 위치 포함)
GPU_ARGS=""
if [ -e /dev/nvidia0 ]; then
    GPU_ARGS="--gpus all"
    echo "[run] GPU: --gpus all (/dev/nvidia0 detected)"
elif docker info 2>/dev/null | grep -qE "(Runtimes|runtimes).*nvidia"; then
    GPU_ARGS="--gpus all"
    echo "[run] GPU: --gpus all (nvidia runtime in docker info)"
elif command -v nvidia-smi &>/dev/null || [ -x /usr/bin/nvidia-smi ] || [ -x /usr/local/nvidia/bin/nvidia-smi ]; then
    GPU_ARGS="--gpus all"
    echo "[run] GPU: --gpus all (nvidia-smi found)"
else
    echo "[run] WARNING: No NVIDIA GPU support detected."
    echo "      cuMotion requires CUDA. To enable GPU:"
    echo "      sudo apt install nvidia-container-toolkit"
    echo "      sudo nvidia-ctk runtime configure --runtime=docker"
    echo "      sudo systemctl restart docker"
fi

# --- X11 display forwarding ---
DISPLAY_ARGS=()
if [ -n "${DISPLAY:-}" ]; then
    xhost +local:root 2>/dev/null || true

    DISPLAY_ARGS+=(
        -e "DISPLAY=${DISPLAY}"
        -e "QT_X11_NO_MITSHM=1"
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"
    )

    if [ -n "${XAUTHORITY:-}" ]; then
        DISPLAY_ARGS+=(
            -e "XAUTHORITY=/tmp/.Xauthority"
            -v "${XAUTHORITY}:/tmp/.Xauthority:ro"
        )
    fi

    echo "[run] X11 forwarding enabled (DISPLAY=${DISPLAY})"
else
    echo "[run] WARNING: No DISPLAY set. GUI apps (RViz) will not work."
fi

# =============================================================================
# Robot device access (USB serial, cameras, etc.)
# =============================================================================
DEVICE_ARGS=()

# Jetson: always use privileged mode for full hardware access
if [ -f /etc/nv_tegra_release ]; then
    DEVICE_ARGS+=(--privileged -v "/dev:/dev")
    echo "[run] Jetson platform detected. Privileged mode enabled."
    MOUNT_DEVICES=false  # skip individual device mounting
fi

if [ "${MOUNT_DEVICES}" = true ]; then
    echo "[run] Scanning robot devices..."

    # --- USB-to-Serial adapters (e.g. robot controllers, grippers) ---
    for dev in /dev/ttyUSB*; do
        if [ -e "${dev}" ]; then
            DEVICE_ARGS+=(--device "${dev}:${dev}")
            echo "       + ${dev} (USB serial)"
        fi
    done

    # --- ACM devices (e.g. Arduino, STM32, USB-CDC) ---
    for dev in /dev/ttyACM*; do
        if [ -e "${dev}" ]; then
            DEVICE_ARGS+=(--device "${dev}:${dev}")
            echo "       + ${dev} (ACM serial)"
        fi
    done

    # --- Video/camera devices (e.g. RealSense, webcam) ---
    for dev in /dev/video*; do
        if [ -e "${dev}" ]; then
            DEVICE_ARGS+=(--device "${dev}:${dev}")
            echo "       + ${dev} (camera)"
        fi
    done

    # --- Joystick/gamepad (e.g. for teleoperation) ---
    for dev in /dev/input/js*; do
        if [ -e "${dev}" ]; then
            DEVICE_ARGS+=(--device "${dev}:${dev}")
            echo "       + ${dev} (joystick)"
        fi
    done

    # --- CAN bus (e.g. robot arm CAN interface) ---
    # CAN is network-based, handled via --network host

    # --- USB bus access for hot-pluggable devices (e.g. RealSense) ---
    if [ -d /dev/bus/usb ]; then
        DEVICE_ARGS+=(-v "/dev/bus/usb:/dev/bus/usb")
        echo "       + /dev/bus/usb (USB bus for hot-plug)"
    fi

    if [ ${#DEVICE_ARGS[@]} -eq 0 ]; then
        echo "       (no devices found)"
    fi
fi

# --- Network (host mode for ROS2 DDS discovery) ---
NETWORK_ARGS="--network host"

# =============================================================================
# Volume mounts
# =============================================================================
# Create persistent build directories on host
mkdir -p "${WORKSPACE_DIR}/.docker/build"
mkdir -p "${WORKSPACE_DIR}/.docker/install"
mkdir -p "${WORKSPACE_DIR}/.docker/log"
mkdir -p "${WORKSPACE_DIR}/.docker/claude"

VOLUME_ARGS=(
    # ---------------------------------------------------------------
    # TAMP_DEV project root -> container workspace
    # ---------------------------------------------------------------
    -v "${WORKSPACE_DIR}:/workspaces/tamp_ws/src/tamp_dev:rw"

    # ---------------------------------------------------------------
    # Persist colcon build artifacts across container restarts
    # ---------------------------------------------------------------
    -v "${WORKSPACE_DIR}/.docker/build:/workspaces/tamp_ws/build:rw"
    -v "${WORKSPACE_DIR}/.docker/install:/workspaces/tamp_ws/install:rw"
    -v "${WORKSPACE_DIR}/.docker/log:/workspaces/tamp_ws/log:rw"

    # ---------------------------------------------------------------
    # Persist Claude Code session history across container restarts
    # ---------------------------------------------------------------
    -v "${WORKSPACE_DIR}/.docker/claude:/root/.claude:rw"

    # ---------------------------------------------------------------
    # System
    # ---------------------------------------------------------------
    -v "/etc/localtime:/etc/localtime:ro"
)

# --- Run container ---
FULL_IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"

echo "============================================"
echo "  Starting container: ${CONTAINER_NAME}"
echo "  Image:     ${FULL_IMAGE}"
echo "  Workspace: ${WORKSPACE_DIR}"
echo "  -> /workspaces/tamp_ws/src/tamp_dev"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "============================================"
echo ""

docker run -it \
    --name "${CONTAINER_NAME}" \
    ${GPU_ARGS} \
    ${NETWORK_ARGS} \
    "${DEVICE_ARGS[@]}" \
    "${DISPLAY_ARGS[@]}" \
    "${VOLUME_ARGS[@]}" \
    -e "TERM=xterm-256color" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    "${FULL_IMAGE}" \
    ${USER_CMD:-bash}
