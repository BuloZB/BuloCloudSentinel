"""
Indoor Drone System API

This module provides a FastAPI-based REST API for the Indoor Drone System microservice.
It serves as the main interface between the BuloCloud Sentinel platform and the indoor drone system.
"""

import os
import logging
import asyncio
import json
from typing import List, Dict, Any, Optional
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, Depends, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# Import ROS2 modules
import rclpy
from rclpy.node import Node

# Import local modules
from ros_bridge import ROSBridge
from models import DroneState, MissionPlan, SensorData, MapData

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="Indoor Drone System API",
    description="API for controlling and monitoring indoor drones with LiDAR and visual positioning",
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

# Initialize ROS2 bridge
ros_bridge = None

@app.on_event("startup")
async def startup_event():
    """Initialize ROS2 and connect to services on startup."""
    global ros_bridge
    
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS bridge
    ros_bridge = ROSBridge()
    
    # Start ROS2 spinning in a separate thread
    asyncio.create_task(ros_bridge.spin())
    
    logger.info("Indoor Drone System API started")

@app.on_event("shutdown")
async def shutdown_event():
    """Clean up resources on shutdown."""
    global ros_bridge
    
    if ros_bridge:
        await ros_bridge.shutdown()
    
    # Shutdown ROS2
    rclpy.shutdown()
    
    logger.info("Indoor Drone System API shutdown")

# API Endpoints

@app.get("/")
async def root():
    """Root endpoint that returns basic service information."""
    return {
        "name": "Indoor Drone System API",
        "version": "0.1.0",
        "status": "running",
    }

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    global ros_bridge
    
    if not ros_bridge or not ros_bridge.is_healthy():
        raise HTTPException(status_code=503, detail="Service unhealthy")
    
    return {"status": "healthy"}

@app.get("/drones", response_model=List[DroneState])
async def get_drones():
    """Get all available drones and their states."""
    global ros_bridge
    
    try:
        drones = await ros_bridge.get_drones()
        return drones
    except Exception as e:
        logger.error(f"Error getting drones: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting drones: {str(e)}")

@app.get("/drones/{drone_id}", response_model=DroneState)
async def get_drone(drone_id: str):
    """Get a specific drone by ID."""
    global ros_bridge
    
    try:
        drone = await ros_bridge.get_drone(drone_id)
        if not drone:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        return drone
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting drone {drone_id}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting drone {drone_id}: {str(e)}")

@app.post("/drones/{drone_id}/command")
async def send_command(drone_id: str, command: Dict[str, Any]):
    """Send a command to a drone."""
    global ros_bridge
    
    try:
        success = await ros_bridge.send_command(drone_id, command)
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to send command to drone {drone_id}")
        return {"success": True}
    except Exception as e:
        logger.error(f"Error sending command to drone {drone_id}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error sending command to drone {drone_id}: {str(e)}")

@app.post("/missions")
async def create_mission(mission: MissionPlan):
    """Create a new mission plan."""
    global ros_bridge
    
    try:
        mission_id = await ros_bridge.create_mission(mission)
        return {"mission_id": mission_id}
    except Exception as e:
        logger.error(f"Error creating mission: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error creating mission: {str(e)}")

@app.get("/missions/{mission_id}")
async def get_mission(mission_id: str):
    """Get a specific mission by ID."""
    global ros_bridge
    
    try:
        mission = await ros_bridge.get_mission(mission_id)
        if not mission:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        return mission
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting mission {mission_id}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting mission {mission_id}: {str(e)}")

@app.post("/missions/{mission_id}/execute")
async def execute_mission(mission_id: str, drone_id: str):
    """Execute a mission with a specific drone."""
    global ros_bridge
    
    try:
        success = await ros_bridge.execute_mission(mission_id, drone_id)
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to execute mission {mission_id} with drone {drone_id}")
        return {"success": True}
    except Exception as e:
        logger.error(f"Error executing mission {mission_id}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error executing mission {mission_id}: {str(e)}")

@app.get("/maps")
async def get_maps():
    """Get all available maps."""
    global ros_bridge
    
    try:
        maps = await ros_bridge.get_maps()
        return maps
    except Exception as e:
        logger.error(f"Error getting maps: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting maps: {str(e)}")

@app.get("/maps/{map_id}")
async def get_map(map_id: str):
    """Get a specific map by ID."""
    global ros_bridge
    
    try:
        map_data = await ros_bridge.get_map(map_id)
        if not map_data:
            raise HTTPException(status_code=404, detail=f"Map {map_id} not found")
        return map_data
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting map {map_id}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting map {map_id}: {str(e)}")

@app.post("/maps/create")
async def create_map(background_tasks: BackgroundTasks, drone_id: str, map_name: str):
    """Start creating a new map with a specific drone."""
    global ros_bridge
    
    try:
        map_id = await ros_bridge.start_mapping(drone_id, map_name)
        background_tasks.add_task(ros_bridge.monitor_mapping, map_id)
        return {"map_id": map_id}
    except Exception as e:
        logger.error(f"Error starting mapping: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error starting mapping: {str(e)}")

@app.websocket("/ws/telemetry/{drone_id}")
async def websocket_telemetry(websocket: WebSocket, drone_id: str):
    """WebSocket endpoint for real-time telemetry data."""
    global ros_bridge
    
    await websocket.accept()
    
    try:
        # Subscribe to telemetry updates
        subscription = await ros_bridge.subscribe_telemetry(drone_id)
        
        while True:
            # Get telemetry data
            telemetry = await subscription.get()
            
            # Send telemetry data to client
            await websocket.send_json(telemetry)
            
            # Wait a short time to avoid flooding
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        logger.info(f"Client disconnected from telemetry stream for drone {drone_id}")
    except Exception as e:
        logger.error(f"Error in telemetry websocket for drone {drone_id}: {str(e)}")
        await websocket.close(code=1011, reason=f"Error: {str(e)}")
    finally:
        # Unsubscribe from telemetry updates
        if 'subscription' in locals():
            await ros_bridge.unsubscribe_telemetry(subscription)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8050, reload=True)
