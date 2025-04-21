"""
SentinelWeb - BuloCloudSentinel Web Interface

This module provides a web interface for BuloCloudSentinel, based on OpenWebUI.
"""

import logging
import os
import sys
from contextlib import asynccontextmanager
from pathlib import Path

# Import SentinelWeb modules
from sentinel_web.drone_adapter.main import DroneAdapter
from sentinel_web.mission_adapter.main import MissionAdapter
from sentinel_web.telemetry_adapter.main import TelemetryAdapter
from sentinel_web.video_adapter.main import VideoAdapter

# Setup logging
logging.basicConfig(stream=sys.stdout, level=logging.INFO)
log = logging.getLogger(__name__)

# Print SentinelWeb banner
print(
    r"""
███████╗███████╗███╗   ██╗████████╗██╗███╗   ██╗███████╗██╗     ██╗    ██╗███████╗██████╗
██╔════╝██╔════╝████╗  ██║╚══██╔══╝██║████╗  ██║██╔════╝██║     ██║    ██║██╔════╝██╔══██╗
███████╗█████╗  ██╔██╗ ██║   ██║   ██║██╔██╗ ██║█████╗  ██║     ██║ █╗ ██║█████╗  ██████╔╝
╚════██║██╔══╝  ██║╚██╗██║   ██║   ██║██║╚██╗██║██╔══╝  ██║     ██║███╗██║██╔══╝  ██╔══██╗
███████║███████╗██║ ╚████║   ██║   ██║██║ ╚████║███████╗███████╗╚███╔███╔╝███████╗██████╔╝
╚══════╝╚══════╝╚═╝  ╚═══╝   ╚═╝   ╚═╝╚═╝  ╚═══╝╚══════╝╚══════╝ ╚══╝╚══╝ ╚══════╝╚═════╝
                                                                                          
BuloCloudSentinel Web Interface - Powered by OpenWebUI
"""
)

# Import OpenWebUI app after banner to avoid duplicate banners
try:
    from open_webui.main import app
    
    # Override the app name
    app.title = "SentinelWeb"
    app.state.WEBUI_NAME = "SentinelWeb"
    
    # Add SentinelWeb initialization to the lifespan
    original_lifespan = app.router.lifespan_context
    
    @asynccontextmanager
    async def sentinel_lifespan(app):
        # Initialize SentinelWeb adapters
        log.info("Initializing SentinelWeb adapters")
        
        # Get configuration from environment variables
        sentinel_api_url = os.environ.get("SENTINEL_API_URL", "http://bulocloud-sentinel-api:8000")
        sentinel_api_token = os.environ.get("SENTINEL_API_TOKEN", "")
        rtmp_server = os.environ.get("RTMP_SERVER", "rtmp://rtmp-server:1935")
        
        # Initialize adapters
        app.state.drone_adapter = DroneAdapter(sentinel_api_url, sentinel_api_token)
        app.state.mission_adapter = MissionAdapter(sentinel_api_url, sentinel_api_token)
        app.state.telemetry_adapter = TelemetryAdapter(sentinel_api_url, sentinel_api_token)
        app.state.video_adapter = VideoAdapter(sentinel_api_url, rtmp_server, sentinel_api_token)
        
        # Create data directory if it doesn't exist
        data_dir = Path("/app/backend/data")
        data_dir.mkdir(parents=True, exist_ok=True)
        
        # Call the original lifespan
        async with original_lifespan(app):
            yield
        
        # Cleanup resources
        log.info("Shutting down SentinelWeb")
    
    # Replace the lifespan
    app.router.lifespan_context = sentinel_lifespan
    
    # Create API routers for SentinelWeb
    from fastapi import APIRouter, Depends, HTTPException
    
    # Create routers
    drones_router = APIRouter()
    missions_router = APIRouter()
    telemetry_router = APIRouter()
    video_router = APIRouter()
    
    # Drones endpoints
    @drones_router.get("/")
    async def get_drones():
        """Get all drones."""
        return {"message": "List of drones will be returned here"}
    
    @drones_router.get("/{drone_id}")
    async def get_drone(drone_id: str):
        """Get a specific drone by ID."""
        return {"message": f"Drone {drone_id} details will be returned here"}
    
    # Missions endpoints
    @missions_router.get("/")
    async def get_missions():
        """Get all missions."""
        return {"message": "List of missions will be returned here"}
    
    @missions_router.get("/{mission_id}")
    async def get_mission(mission_id: str):
        """Get a specific mission by ID."""
        return {"message": f"Mission {mission_id} details will be returned here"}
    
    # Telemetry endpoints
    @telemetry_router.get("/{drone_id}")
    async def get_telemetry(drone_id: str):
        """Get telemetry for a specific drone."""
        return {"message": f"Telemetry for drone {drone_id} will be returned here"}
    
    # Video endpoints
    @video_router.get("/streams")
    async def get_video_streams():
        """Get all video streams."""
        return {"message": "List of video streams will be returned here"}
    
    @video_router.get("/{drone_id}")
    async def get_drone_stream(drone_id: str):
        """Get video stream for a specific drone."""
        return {"message": f"Video stream for drone {drone_id} will be returned here"}
    
    # Include SentinelWeb routers
    app.include_router(drones_router, prefix="/api/drones", tags=["drones"])
    app.include_router(missions_router, prefix="/api/missions", tags=["missions"])
    app.include_router(telemetry_router, prefix="/api/telemetry", tags=["telemetry"])
    app.include_router(video_router, prefix="/api/video", tags=["video"])
    
    log.info("SentinelWeb initialized successfully")
    
except ImportError:
    log.error("Failed to import OpenWebUI. Make sure it's installed correctly.")
    raise
