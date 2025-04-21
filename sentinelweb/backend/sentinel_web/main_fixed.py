"""
SentinelWeb - BuloCloudSentinel Web Interface

This module provides a web interface for BuloCloudSentinel, based on OpenWebUI.
It imports the OpenWebUI app and extends it with drone-specific functionality.
"""

import logging
import os
import sys
from contextlib import asynccontextmanager
from pathlib import Path

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

# Import OpenWebUI app
try:
    # Import the app from OpenWebUI
    from open_webui.main import app
    
    # Import SentinelWeb modules
    from sentinel_web.drone_adapter.main import DroneAdapter
    from sentinel_web.mission_adapter.main import MissionAdapter
    from sentinel_web.telemetry_adapter.main import TelemetryAdapter
    from sentinel_web.video_adapter.main import VideoAdapter
    
    # Import FastAPI components
    from fastapi import APIRouter, Depends, HTTPException, Request
    from typing import List, Dict, Any, Optional
    
    # Override the app name
    app.title = "SentinelWeb"
    app.state.WEBUI_NAME = "SentinelWeb"
    
    # Store the original lifespan
    original_lifespan = app.router.lifespan_context
    
    @asynccontextmanager
    async def sentinel_lifespan(app):
        """Extended lifespan that initializes SentinelWeb adapters."""
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
    drones_router = APIRouter()
    missions_router = APIRouter()
    telemetry_router = APIRouter()
    video_router = APIRouter()
    
    # Drones endpoints
    @drones_router.get("/")
    async def get_drones(request: Request) -> List[Dict[str, Any]]:
        """Get all drones."""
        try:
            return await request.app.state.drone_adapter.get_drones()
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Error getting drones: {str(e)}")
    
    @drones_router.get("/{drone_id}")
    async def get_drone(drone_id: str, request: Request) -> Dict[str, Any]:
        """Get a specific drone by ID."""
        drone = await request.app.state.drone_adapter.get_drone(drone_id)
        if not drone:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        return drone
    
    @drones_router.post("/{drone_id}/command")
    async def send_command(
        drone_id: str, 
        command: str, 
        params: Optional[Dict[str, Any]] = None, 
        request: Request
    ) -> Dict[str, bool]:
        """Send a command to a drone."""
        # Check if drone exists
        drone = await request.app.state.drone_adapter.get_drone(drone_id)
        if not drone:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        
        # Send command
        success = await request.app.state.drone_adapter.send_command(drone_id, command, params)
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to send command to drone {drone_id}")
        
        return {"success": True}
    
    # Missions endpoints
    @missions_router.get("/")
    async def get_missions(request: Request) -> List[Dict[str, Any]]:
        """Get all missions."""
        try:
            return await request.app.state.mission_adapter.get_missions()
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Error getting missions: {str(e)}")
    
    @missions_router.get("/{mission_id}")
    async def get_mission(mission_id: str, request: Request) -> Dict[str, Any]:
        """Get a specific mission by ID."""
        mission = await request.app.state.mission_adapter.get_mission(mission_id)
        if not mission:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        return mission
    
    @missions_router.post("/")
    async def create_mission(mission_data: Dict[str, Any], request: Request) -> Dict[str, Any]:
        """Create a new mission."""
        try:
            mission = await request.app.state.mission_adapter.create_mission(mission_data)
            if not mission:
                raise HTTPException(status_code=500, detail="Failed to create mission")
            return mission
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Error creating mission: {str(e)}")
    
    # Telemetry endpoints
    @telemetry_router.get("/{drone_id}")
    async def get_telemetry(drone_id: str, request: Request) -> Dict[str, Any]:
        """Get telemetry for a specific drone."""
        telemetry = await request.app.state.telemetry_adapter.get_telemetry(drone_id)
        if not telemetry:
            raise HTTPException(status_code=404, detail=f"Telemetry for drone {drone_id} not found")
        return telemetry
    
    @telemetry_router.get("/{drone_id}/battery")
    async def get_battery_status(drone_id: str, request: Request) -> Dict[str, Any]:
        """Get battery status for a specific drone."""
        battery = await request.app.state.telemetry_adapter.get_battery_status(drone_id)
        if not battery:
            raise HTTPException(status_code=404, detail=f"Battery status for drone {drone_id} not found")
        return battery
    
    # Video endpoints
    @video_router.get("/streams")
    async def get_video_streams(request: Request) -> List[Dict[str, Any]]:
        """Get all video streams."""
        try:
            return await request.app.state.video_adapter.get_video_streams()
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Error getting video streams: {str(e)}")
    
    @video_router.get("/drones/{drone_id}")
    async def get_drone_stream(drone_id: str, request: Request) -> Dict[str, Any]:
        """Get video stream for a specific drone."""
        stream = await request.app.state.video_adapter.get_drone_stream(drone_id)
        if not stream:
            raise HTTPException(status_code=404, detail=f"Video stream for drone {drone_id} not found")
        return stream
    
    # Include SentinelWeb routers
    app.include_router(drones_router, prefix="/api/drones", tags=["drones"])
    app.include_router(missions_router, prefix="/api/missions", tags=["missions"])
    app.include_router(telemetry_router, prefix="/api/telemetry", tags=["telemetry"])
    app.include_router(video_router, prefix="/api/video", tags=["video"])
    
    log.info("SentinelWeb initialized successfully")
    
except ImportError as e:
    log.error(f"Failed to import OpenWebUI: {str(e)}")
    log.error("Make sure OpenWebUI is installed correctly and in your PYTHONPATH")
    raise
