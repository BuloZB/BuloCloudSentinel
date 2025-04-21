"""
Drones API router for SentinelWeb.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import List, Dict, Any, Optional

from open_webui.internal.auth import get_current_user
from open_webui.models.users import User
from open_webui.sentinel_adapter.main import get_sentinel_adapter
from open_webui.sentinel_adapter.drone_adapter import DroneInfo

router = APIRouter()

@router.get("/", response_model=List[DroneInfo])
async def list_drones(current_user: User = Depends(get_current_user)):
    """
    List all available drones.
    """
    adapter = get_sentinel_adapter()
    drones = await adapter.get_drone_adapter().list_drones()
    return drones

@router.get("/{drone_id}", response_model=DroneInfo)
async def get_drone(drone_id: str, current_user: User = Depends(get_current_user)):
    """
    Get information about a specific drone.
    """
    adapter = get_sentinel_adapter()
    drone = await adapter.get_drone_adapter().get_drone(drone_id)
    
    if not drone:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Drone with ID {drone_id} not found"
        )
    
    return drone

@router.get("/{drone_id}/status")
async def get_drone_status(drone_id: str, current_user: User = Depends(get_current_user)):
    """
    Get the current status of a drone.
    """
    adapter = get_sentinel_adapter()
    status_data = await adapter.get_drone_adapter().get_drone_status(drone_id)
    return status_data

@router.post("/{drone_id}/command")
async def send_drone_command(
    drone_id: str,
    command_data: Dict[str, Any],
    current_user: User = Depends(get_current_user)
):
    """
    Send a command to a drone.
    """
    adapter = get_sentinel_adapter()
    
    command = command_data.get("command")
    parameters = command_data.get("parameters", {})
    
    if not command:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Command is required"
        )
    
    result = await adapter.get_drone_adapter().send_command(drone_id, command, parameters)
    return result
