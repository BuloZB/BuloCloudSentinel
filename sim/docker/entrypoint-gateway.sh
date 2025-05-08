#!/bin/bash
set -e

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Print environment for debugging
echo "Environment:"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "SENTINEL_API_URL=$SENTINEL_API_URL"
echo "GATEWAY_CONFIG=$GATEWAY_CONFIG"

# Check if the gateway config exists
if [ -n "$GATEWAY_CONFIG" ] && [ -f "$GATEWAY_CONFIG" ]; then
    echo "Using gateway config: $GATEWAY_CONFIG"
else
    echo "Gateway config not found: $GATEWAY_CONFIG"
    echo "Using default configuration"
fi

# Execute the command
exec "$@"
