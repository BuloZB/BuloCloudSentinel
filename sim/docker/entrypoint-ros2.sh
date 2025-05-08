#!/bin/bash
set -e

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Check if the workspace has been built
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Print environment for debugging
echo "Environment:"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI=$CYCLONEDDS_URI"

# Check if the workspace needs to be built
if [ -d "/ros2_ws/src" ] && [ "$(ls -A /ros2_ws/src)" ]; then
    echo "Building ROS 2 workspace..."
    cd /ros2_ws
    colcon build --symlink-install
    source /ros2_ws/install/setup.bash
fi

# Execute the command
exec "$@"
