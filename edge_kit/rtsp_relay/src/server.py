#!/usr/bin/env python3
"""
Bulo.CloudSentinel RTSP Relay Server

This module provides a server for relaying RTSP streams to WebRTC and HLS.
It ingests local RTSP streams and publishes low-bitrate HLS/WebRTC to the central cloud.
"""

import os
import sys
import logging
import json
import time
import asyncio
import signal
import subprocess
import threading
import yaml
from typing import Dict, List, Any, Optional, Union, Tuple
from enum import Enum
from pathlib import Path
import uuid

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
from pydantic import BaseModel, Field

# Import stream manager
from stream_manager import StreamManager, StreamInfo, StreamStatus

# Configure logging
log_level = os.environ.get("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, log_level),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("rtsp_relay")

# Create FastAPI app
app = FastAPI(
    title="Bulo.CloudSentinel RTSP Relay",
    description="RTSP relay server for Bulo.CloudSentinel",
    version="0.1.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Create static directory for HLS segments
os.makedirs("/storage/hls", exist_ok=True)
app.mount("/hls", StaticFiles(directory="/storage/hls"), name="hls")

# Create stream manager
rtsp_sources_config = os.environ.get("RTSP_SOURCES_CONFIG", "/config/rtsp_sources.yaml")
hls_segment_duration = int(os.environ.get("HLS_SEGMENT_DURATION", "2"))
webrtc_ice_servers = os.environ.get("WEBRTC_ICE_SERVERS", "stun:stun.l.google.com:19302")
stream_buffer_size = int(os.environ.get("STREAM_BUFFER_SIZE", "1024"))

# Initialize stream manager
stream_manager = StreamManager(
    rtsp_sources_config=rtsp_sources_config,
    hls_segment_duration=hls_segment_duration,
    webrtc_ice_servers=webrtc_ice_servers,
    stream_buffer_size=stream_buffer_size,
)

# API models
class StreamListResponse(BaseModel):
    streams: List[StreamInfo]

class StreamResponse(BaseModel):
    stream: StreamInfo

class HealthResponse(BaseModel):
    status: str
    version: str
    streams: int
    active_connections: int

class RTSPSourceRequest(BaseModel):
    name: str
    url: str
    enabled: bool = True

# API routes
@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Check the health of the relay server."""
    return {
        "status": "ok",
        "version": "0.1.0",
        "streams": len(stream_manager.streams),
        "active_connections": stream_manager.active_connections,
    }

@app.get("/streams", response_model=StreamListResponse)
async def list_streams():
    """List all available streams."""
    streams = await stream_manager.list_streams()
    return {"streams": streams}

@app.get("/streams/{stream_id}", response_model=StreamResponse)
async def get_stream(stream_id: str):
    """Get information about a specific stream."""
    try:
        stream = await stream_manager.get_stream(stream_id)
        return {"stream": stream}
    except Exception as e:
        raise HTTPException(status_code=404, detail=f"Stream not found: {str(e)}")

@app.post("/streams", response_model=StreamResponse)
async def add_stream(request: RTSPSourceRequest):
    """Add a new RTSP stream."""
    try:
        stream = await stream_manager.add_stream(request.name, request.url, request.enabled)
        return {"stream": stream}
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to add stream: {str(e)}")

@app.delete("/streams/{stream_id}")
async def remove_stream(stream_id: str):
    """Remove a stream."""
    try:
        await stream_manager.remove_stream(stream_id)
        return {"status": "ok"}
    except Exception as e:
        raise HTTPException(status_code=404, detail=f"Stream not found: {str(e)}")

@app.post("/streams/{stream_id}/start")
async def start_stream(stream_id: str):
    """Start a stream."""
    try:
        await stream_manager.start_stream(stream_id)
        return {"status": "ok"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to start stream: {str(e)}")

@app.post("/streams/{stream_id}/stop")
async def stop_stream(stream_id: str):
    """Stop a stream."""
    try:
        await stream_manager.stop_stream(stream_id)
        return {"status": "ok"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Failed to stop stream: {str(e)}")

@app.get("/streams/{stream_id}/hls")
async def get_hls_playlist(stream_id: str):
    """Get HLS playlist for a stream."""
    try:
        playlist_path = await stream_manager.get_hls_playlist(stream_id)
        return FileResponse(playlist_path)
    except Exception as e:
        raise HTTPException(status_code=404, detail=f"HLS playlist not found: {str(e)}")

@app.get("/streams/{stream_id}/hls/player")
async def get_hls_player(stream_id: str):
    """Get HLS player for a stream."""
    try:
        stream = await stream_manager.get_stream(stream_id)
        
        # Create a simple HLS player
        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>HLS Player - {stream.name}</title>
            <script src="https://cdn.jsdelivr.net/npm/hls.js@latest"></script>
            <style>
                body {{ margin: 0; padding: 0; background-color: #000; }}
                video {{ width: 100%; height: 100vh; }}
            </style>
        </head>
        <body>
            <video id="video" controls autoplay></video>
            <script>
                var video = document.getElementById('video');
                var videoSrc = '/streams/{stream_id}/hls';
                
                if (Hls.isSupported()) {{
                    var hls = new Hls();
                    hls.loadSource(videoSrc);
                    hls.attachMedia(video);
                    hls.on(Hls.Events.MANIFEST_PARSED, function() {{
                        video.play();
                    }});
                }}
                else if (video.canPlayType('application/vnd.apple.mpegurl')) {{
                    video.src = videoSrc;
                    video.addEventListener('loadedmetadata', function() {{
                        video.play();
                    }});
                }}
            </script>
        </body>
        </html>
        """
        
        return HTMLResponse(content=html_content)
    except Exception as e:
        raise HTTPException(status_code=404, detail=f"Stream not found: {str(e)}")

@app.websocket("/streams/{stream_id}/webrtc")
async def webrtc_endpoint(websocket: WebSocket, stream_id: str):
    """WebRTC endpoint for a stream."""
    try:
        await websocket.accept()
        
        # Register connection
        connection_id = str(uuid.uuid4())
        await stream_manager.register_webrtc_connection(stream_id, connection_id, websocket)
        
        try:
            # Handle WebRTC signaling
            while True:
                data = await websocket.receive_text()
                message = json.loads(data)
                
                # Handle message
                if message.get("type") == "offer":
                    # Handle SDP offer
                    await stream_manager.handle_webrtc_offer(stream_id, connection_id, message)
                elif message.get("type") == "ice-candidate":
                    # Handle ICE candidate
                    await stream_manager.handle_webrtc_ice_candidate(stream_id, connection_id, message)
                
        except WebSocketDisconnect:
            # Unregister connection
            await stream_manager.unregister_webrtc_connection(stream_id, connection_id)
            
    except Exception as e:
        logger.error(f"WebRTC error: {str(e)}")
        await websocket.close()

@app.get("/metrics")
async def metrics():
    """Get metrics from the relay server."""
    try:
        metrics_data = await stream_manager.get_metrics()
        return JSONResponse(content=metrics_data)
    except Exception as e:
        logger.error(f"Metrics error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Metrics error: {str(e)}")

# Shutdown handler
@app.on_event("shutdown")
async def shutdown_event():
    """Shutdown event handler."""
    await stream_manager.shutdown()

# Main function
def main():
    """Run the relay server."""
    # Start stream manager
    asyncio.create_task(stream_manager.start())
    
    # Start server
    port = int(os.environ.get("PORT", "8888"))
    uvicorn.run(app, host="0.0.0.0", port=port)

if __name__ == "__main__":
    main()
