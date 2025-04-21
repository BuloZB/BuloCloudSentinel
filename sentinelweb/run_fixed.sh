#!/bin/bash

# Set environment variables
export PYTHONPATH=$PYTHONPATH:$(pwd):$(pwd)/open-webui:$(pwd)/backend
export SENTINEL_API_URL=${SENTINEL_API_URL:-http://localhost:8000}
export SENTINEL_API_TOKEN=${SENTINEL_API_TOKEN:-""}
export RTMP_SERVER=${RTMP_SERVER:-rtmp://localhost:1935}
export SECRET_KEY=${SECRET_KEY:-sentinelweb-secret-key}

# Check if OpenWebUI is installed
if [ ! -d "open-webui" ]; then
    echo "OpenWebUI not found. Cloning repository..."
    git clone https://github.com/open-webui/open-webui.git
    
    # Install OpenWebUI dependencies
    cd open-webui/backend
    pip install -e .
    cd ../..
fi

# Install SentinelWeb dependencies
cd backend
pip install -e .
cd ..

# Run the application
python -m uvicorn sentinel_web.main_fixed:app --host 0.0.0.0 --port 8080 --reload
