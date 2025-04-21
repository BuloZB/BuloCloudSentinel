#!/bin/bash

# Apply drone adapter patches to SentinelWeb
echo "Applying drone adapter patches to SentinelWeb..."
python -m backend.sentinel_web.drone_adapter.patch

# Start the application
echo "Starting SentinelWeb..."
cd /app
python -m backend.sentinel_web.main
