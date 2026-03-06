#!/bin/bash
# =============================================================================
# build_image.sh - Build the tamp_dev Docker image
#
# Usage:
#   ./build_image.sh                  # Auto-detect architecture
#   ./build_image.sh --arch amd64     # Force amd64
#   ./build_image.sh --arch arm64     # Force arm64 (Jetson)
#   ./build_image.sh --no-cache       # Rebuild without Docker cache
#   ./build_image.sh --tag v1.0       # Custom tag
#   ./build_image.sh --pull           # Pull base image before build
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# --- Default configuration ---
IMAGE_NAME="tamp_dev"
IMAGE_TAG="latest"

# Load overrides from .env.docker if present
ENV_FILE="${PROJECT_DIR}/.env.docker"
if [ -f "${ENV_FILE}" ]; then
    # shellcheck source=/dev/null
    source "${ENV_FILE}"
fi

# Base image per architecture
declare -A BASE_IMAGES=(
    ["amd64"]="nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_f247dd1051869171c3fc53bb35f6b907"
    ["arm64"]="nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c"
)

# --- Argument parsing ---
ARCH=""
DOCKER_BUILD_ARGS=""
PULL_BASE=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --arch)
            ARCH="$2"
            shift 2
            ;;
        --no-cache)
            DOCKER_BUILD_ARGS="${DOCKER_BUILD_ARGS} --no-cache"
            shift
            ;;
        --tag)
            IMAGE_TAG="$2"
            shift 2
            ;;
        --pull)
            PULL_BASE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --arch ARCH     Architecture: amd64 or arm64 (default: auto-detect)"
            echo "  --no-cache      Build without Docker cache"
            echo "  --tag TAG       Image tag (default: latest)"
            echo "  --pull          Pull base image from NGC before building"
            echo "  -h, --help      Show this help"
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $1"
            echo "Use --help for usage information."
            exit 1
            ;;
    esac
done

# --- Architecture detection ---
if [ -z "${ARCH}" ]; then
    HOST_ARCH="$(uname -m)"
    case "${HOST_ARCH}" in
        x86_64)  ARCH="amd64" ;;
        aarch64) ARCH="arm64" ;;
        *)
            echo "ERROR: Unsupported architecture: ${HOST_ARCH}"
            echo "       Use --arch to specify manually."
            exit 1
            ;;
    esac
    echo "[build] Auto-detected architecture: ${ARCH}"
fi

# --- Validate architecture ---
if [[ ! -v "BASE_IMAGES[${ARCH}]" ]]; then
    echo "ERROR: No base image defined for architecture: ${ARCH}"
    echo "       Supported: ${!BASE_IMAGES[*]}"
    exit 1
fi

BASE_IMAGE="${BASE_IMAGES[${ARCH}]}"
FULL_TAG="${IMAGE_NAME}:${IMAGE_TAG}-${ARCH}"

echo "============================================"
echo "  Building Docker Image"
echo "  Base:  ${BASE_IMAGE}"
echo "  Tag:   ${FULL_TAG}"
echo "  Arch:  ${ARCH}"
echo "============================================"
echo ""

# --- Pull base image if requested ---
if [ "${PULL_BASE}" = true ]; then
    echo "[build] Pulling base image..."
    docker pull "${BASE_IMAGE}"
    echo ""
fi

# --- Build ---
docker build \
    ${DOCKER_BUILD_ARGS} \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    -t "${FULL_TAG}" \
    -t "${IMAGE_NAME}:${IMAGE_TAG}" \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${SCRIPT_DIR}"

echo ""
echo "============================================"
echo "  Build Complete"
echo "  Tagged: ${FULL_TAG}"
echo "  Also:   ${IMAGE_NAME}:${IMAGE_TAG}"
echo "============================================"
echo ""
echo "  Next: ./run_container.sh"
