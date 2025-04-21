#!/bin/bash

# Clone OpenWebUI repository if it doesn't exist
if [ ! -d "temp-openwebui" ]; then
    echo "Cloning OpenWebUI repository..."
    git clone https://github.com/open-webui/open-webui.git temp-openwebui
fi

# Build and run SentinelWeb
echo "Building and running SentinelWeb..."
docker-compose up -d --build

echo "SentinelWeb is now running!"
echo "Access the web interface at http://localhost:3000"
