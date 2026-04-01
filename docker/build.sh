#!/bin/bash
set -e

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Go to project root
cd "$SCRIPT_DIR/.."

# Build the image
echo "Building Docker image..."
docker build -t autoware_off-road_sim:6.0 -f docker/Dockerfile .
