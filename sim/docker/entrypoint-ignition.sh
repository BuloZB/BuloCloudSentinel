#!/bin/bash
set -e

# Print environment for debugging
echo "Environment:"
echo "IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH"
echo "IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH"
echo "IGN_TRANSPORT_TOPIC_STATISTICS=$IGN_TRANSPORT_TOPIC_STATISTICS"
echo "IGN_GUI_PLUGIN_PATH=$IGN_GUI_PLUGIN_PATH"

# Check if a world file is provided
WORLD_FILE=""
if [ -n "$1" ] && [ "$1" != "ign" ]; then
    WORLD_FILE="$1"
    shift
fi

# If a world file is provided, use it
if [ -n "$WORLD_FILE" ]; then
    if [ -f "$WORLD_FILE" ]; then
        echo "Starting Ignition Gazebo with world file: $WORLD_FILE"
        exec ign gazebo "$WORLD_FILE" "$@"
    else
        echo "World file not found: $WORLD_FILE"
        echo "Available world files:"
        find /worlds -name "*.sdf" -o -name "*.world"
        exit 1
    fi
else
    # Otherwise, use the default command
    echo "Starting Ignition Gazebo with default command"
    exec "$@"
fi
