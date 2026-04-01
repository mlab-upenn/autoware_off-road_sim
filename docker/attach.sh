#!/bin/bash
set -e

CONTAINER_NAME="autoware_off-road_sim"

echo "Attaching to running container: $CONTAINER_NAME..."
echo "Automatically sourcing ROS 2 environment..."

# Execute an interactive bash shell that inherits the ROS 2 setup environment natively from the project workspace
docker exec -w /workspace/autoware_off-road_sim -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && exec bash"
