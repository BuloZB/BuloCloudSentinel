#!/bin/bash

# Apply SentinelWeb patches to OpenWebUI
echo "Applying SentinelWeb patches to OpenWebUI..."
python -m backend.sentinel_adapter.patch

# Start the application
echo "Starting SentinelWeb..."
cd /app
python -m backend.open_webui.main
