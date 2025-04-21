@echo off

REM Set environment variables
set PYTHONPATH=%PYTHONPATH%;%CD%;%CD%\open-webui;%CD%\backend
set SENTINEL_API_URL=http://localhost:8000
set SENTINEL_API_TOKEN=
set RTMP_SERVER=rtmp://localhost:1935
set SECRET_KEY=sentinelweb-secret-key

REM Check if OpenWebUI is installed
if not exist "open-webui" (
    echo OpenWebUI not found. Cloning repository...
    git clone https://github.com/open-webui/open-webui.git
    
    REM Install OpenWebUI dependencies
    cd open-webui\backend
    pip install -e .
    cd ..\..
)

REM Install SentinelWeb dependencies
cd backend
pip install -e .
cd ..

REM Run the application
python -m uvicorn sentinel_web.main_fixed:app --host 0.0.0.0 --port 8080 --reload
