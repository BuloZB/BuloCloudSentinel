@echo off
echo Building and running SentinelWeb...

REM Clone OpenWebUI repository if it doesn't exist
if not exist temp-openwebui (
    echo Cloning OpenWebUI repository...
    git clone https://github.com/open-webui/open-webui.git temp-openwebui
)

REM Build and run SentinelWeb
docker-compose up -d --build

echo SentinelWeb is now running!
echo Access the web interface at http://localhost:3000
