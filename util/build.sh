#!/bin/bash

# Build script for TrajectorySense ROS 2 Docker container
# This script builds the Docker image with all necessary ROS 2 and Gazebo packages

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
DOCKERFILE_NAME="full.dockerfile"
DOCKERFILE_PATH="../"  # Dockerfile is in parent directory

echo -e "${BLUE}=== TrajectorySense Docker Build Script ===${NC}"
echo -e "${YELLOW}Building Docker image: ${IMAGE_NAME}:${IMAGE_TAG}${NC}"

# Check if Docker is running
if ! docker info >/dev/null 2>&1; then
    echo -e "${RED}Error: Docker is not running. Please start Docker and try again.${NC}"
    exit 1
fi

# Check if Dockerfile exists
if [ ! -f "../${DOCKERFILE_NAME}" ]; then
    echo -e "${RED}Error: ${DOCKERFILE_NAME} not found in parent directory.${NC}"
    echo -e "${YELLOW}Make sure the ${DOCKERFILE_NAME} is in the repository root.${NC}"
    exit 1
fi

echo -e "${YELLOW}Starting build process...${NC}"

# Build the Docker image
docker build \
    --file "${DOCKERFILE_PATH}${DOCKERFILE_NAME}" \
    --tag "${IMAGE_NAME}:${IMAGE_TAG}" \
    --tag "${IMAGE_NAME}:latest" \
    "${DOCKERFILE_PATH}"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Successfully built Docker image: ${IMAGE_NAME}:${IMAGE_TAG}${NC}"
    echo -e "${BLUE}Image size:${NC}"
    docker images "${IMAGE_NAME}:${IMAGE_TAG}" --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}"
    echo -e "${YELLOW}To run the container, use: ./run_docker.sh${NC}"
else
    echo -e "${RED}✗ Failed to build Docker image${NC}"
    exit 1
fi