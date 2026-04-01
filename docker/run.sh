#!/bin/bash
set -e

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Go to project root
cd "$SCRIPT_DIR/.."

# Go to project root
cd "$SCRIPT_DIR/.."

# Check if nvidia-smi is available
if ! command -v nvidia-smi &> /dev/null; then
    echo "Error: nvidia-smi not found. Please ensure NVIDIA drivers are installed."
    exit 1
fi

# Prepare X11 access
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi
chmod 777 $XAUTH

# Run the container
echo "Running Docker container..."
docker run --rm -it \
    --gpus all \
    --network host \
    --workdir /workspace/autoware_off-road_sim \
    -e "DISPLAY=${DISPLAY}" \
    -e "XAUTHORITY=${XAUTH}" \
    -e "NVIDIA_DRIVER_CAPABILITIES=all" \
    -e "NVIDIA_VISIBLE_DEVICES=all" \
    -v $XAUTH:$XAUTH \
    -v $(pwd)/docker/entrypoint.sh:/usr/local/bin/entrypoint.sh \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/workspace/autoware_off-road_sim \
    --name autoware_off-road_sim \
    autoware_off-road_sim:6.0 \
    "$@"
