"""
Drone API routes for the Drone Swarm System.

This module provides endpoints for drone management and control.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, BackgroundTasks
from typing import List, Dict, Any, Optional
from pydantic import BaseModel

# Import local modules
from services.drone_service import DroneService
from api.schemas.drone import (
    Drone,
    DroneCreate,
    DroneUpdate,
    DroneState,
    DroneCommand,
    DroneCapabilities
)

router = APIRouter()

# Models
class CommandRequest(BaseModel):
    """Request model for sending a command to a drone."""
    command: str
    parameters: Optional[Dict[str, Any]] = None

class TelemetryRequest(BaseModel):
    """Request model for configuring telemetry."""
    enabled: bool
    frequency: Optional[float] = None  # Hz
    items: Optional[List[str]] = None

# Helper functions
def get_drone_service(request: Request) -> DroneService:
    """Get the drone service from the app state."""
    return request.app.state.drone_service

# Routes
@router.get("/")
async def get_all_drones(
    status: Optional[str] = None,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Get all drones, optionally filtered by status."""
    try:
        drones = await drone_service.get_all_drones(status)
        return drones
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting drones: {str(e)}")

@router.get("/{drone_id}")
async def get_drone(
    drone_id: str,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Get a specific drone by ID."""
    try:
        drone = await drone_service.get_drone(drone_id)
        if not drone:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        return drone
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting drone: {str(e)}")

@router.post("/")
async def create_drone(
    drone: DroneCreate,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Create a new drone."""
    try:
        new_drone = await drone_service.create_drone(drone)
        return new_drone
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating drone: {str(e)}")

@router.put("/{drone_id}")
async def update_drone(
    drone_id: str,
    drone: DroneUpdate,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Update a drone."""
    try:
        updated_drone = await drone_service.update_drone(drone_id, drone)
        if not updated_drone:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        return updated_drone
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating drone: {str(e)}")

@router.delete("/{drone_id}")
async def delete_drone(
    drone_id: str,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Delete a drone."""
    try:
        success = await drone_service.delete_drone(drone_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting drone: {str(e)}")

@router.get("/{drone_id}/state")
async def get_drone_state(
    drone_id: str,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Get the current state of a drone."""
    try:
        state = await drone_service.get_drone_state(drone_id)
        if not state:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        return state
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting drone state: {str(e)}")

@router.post("/{drone_id}/command")
async def send_command(
    drone_id: str,
    command: CommandRequest,
    background_tasks: BackgroundTasks,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Send a command to a drone."""
    try:
        # Check if drone exists
        drone = await drone_service.get_drone(drone_id)
        if not drone:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        
        # Send command in background
        background_tasks.add_task(
            drone_service.send_command,
            drone_id,
            command.command,
            command.parameters
        )
        
        return {"message": f"Command {command.command} sent to drone {drone_id}"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error sending command: {str(e)}")

@router.get("/{drone_id}/capabilities")
async def get_drone_capabilities(
    drone_id: str,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Get the capabilities of a drone."""
    try:
        capabilities = await drone_service.get_drone_capabilities(drone_id)
        if not capabilities:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        return capabilities
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting drone capabilities: {str(e)}")

@router.post("/{drone_id}/telemetry")
async def configure_telemetry(
    drone_id: str,
    config: TelemetryRequest,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Configure telemetry for a drone."""
    try:
        # Check if drone exists
        drone = await drone_service.get_drone(drone_id)
        if not drone:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        
        # Configure telemetry
        success = await drone_service.configure_telemetry(
            drone_id,
            config.enabled,
            config.frequency,
            config.items
        )
        
        return {"success": success}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error configuring telemetry: {str(e)}")

@router.post("/{drone_id}/connect")
async def connect_drone(
    drone_id: str,
    background_tasks: BackgroundTasks,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Connect to a drone."""
    try:
        # Check if drone exists
        drone = await drone_service.get_drone(drone_id)
        if not drone:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        
        # Connect to drone in background
        background_tasks.add_task(drone_service.connect_drone, drone_id)
        
        return {"message": f"Connecting to drone {drone_id}"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error connecting to drone: {str(e)}")

@router.post("/{drone_id}/disconnect")
async def disconnect_drone(
    drone_id: str,
    background_tasks: BackgroundTasks,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Disconnect from a drone."""
    try:
        # Check if drone exists
        drone = await drone_service.get_drone(drone_id)
        if not drone:
            raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
        
        # Disconnect from drone in background
        background_tasks.add_task(drone_service.disconnect_drone, drone_id)
        
        return {"message": f"Disconnecting from drone {drone_id}"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error disconnecting from drone: {str(e)}")

@router.get("/{drone_id}/logs")
async def get_drone_logs(
    drone_id: str,
    limit: int = 100,
    level: Optional[str] = None,
    drone_service: DroneService = Depends(get_drone_service)
):
    """Get logs for a drone."""
    try:
        logs = await drone_service.get_drone_logs(drone_id, limit, level)
        return logs
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting drone logs: {str(e)}")
