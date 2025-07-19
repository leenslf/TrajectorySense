#!/bin/bash

# Run script for TrajectorySense ROS 2 Docker container
# This script runs the Docker container with GPU support and proper display forwarding

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
IMAGE_NAME="trajectorysense-ros2"
IMAGE_TAG="humble"
CONTAINER_NAME="trajectorysense-container"
WORKSPACE_PATH="$(dirname "$(pwd)")"  # Parent directory (repository root)

echo -e "${BLUE}=== TrajectorySense Docker Run Script ===${NC}"

# Check if Docker is running
if ! docker info >/dev/null 2>&1; then
    echo -e "${RED}Error: Docker is not running. Please start Docker and try again.${NC}"
    exit 1
fi

# Check if image exists
if ! docker images "${IMAGE_NAME}:${IMAGE_TAG}" --format "{{.Repository}}" | grep -q "${IMAGE_NAME}"; then
    echo -e "${RED}Error: Docker image ${IMAGE_NAME}:${IMAGE_TAG} not found.${NC}"
    echo -e "${YELLOW}Please build the image first using: ./build_docker.sh${NC}"
    exit 1
fi

# Remove existing container if it exists
if docker ps -a --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo -e "${YELLOW}Removing existing container: ${CONTAINER_NAME}${NC}"
    docker rm -f "${CONTAINER_NAME}"
fi

# Enable X11 forwarding for GUI applications
echo -e "${YELLOW}Enabling X11 forwarding for GUI applications...${NC}"
xhost +local:docker

# Check for NVIDIA GPU support
GPU_ARGS=""
if command -v nvidia-smi >/dev/null 2>&1; then
    echo -e "${GREEN}NVIDIA GPU detected. Enabling GPU support...${NC}"
    GPU_ARGS="--gpus all \
              --env NVIDIA_VISIBLE_DEVICES=all \
              --env NVIDIA_DRIVER_CAPABILITIES=all"
else
    echo -e "${YELLOW}No NVIDIA GPU detected. Running without GPU acceleration.${NC}"
fi

echo -e "${YELLOW}Starting container: ${CONTAINER_NAME}${NC}"
echo -e "${BLUE}Workspace mounted at: /workspace/TrajectorySense${NC}"

# Run the Docker container
docker run -it \
    --name "${CONTAINER_NAME}" \
    --hostname trajectorysense-docker \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${WORKSPACE_PATH}:/workspace/TrajectorySense" \
    --device=/dev/dri:/dev/dri \
    --net=host \
    --privileged \
    ${GPU_ARGS} \
    --workdir="/workspace/TrajectorySense" \
    "${IMAGE_NAME}:${IMAGE_TAG}" \
    bash

echo -e "${GREEN}Container session ended.${NC}"

# Optional: Clean up X11 permissions (uncomment if needed)
# xhost -local:docker