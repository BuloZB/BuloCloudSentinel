"""
Drones API router for SentinelWeb.
"""

from fastapi import APIRouter, Depends, HTTPException, status

from security.validation.unified_validation import (
    validate_email,
    validate_username,
    validate_name,
    validate_uuid,
    validate_url,
    sanitize_string,
    sanitize_html,
    check_sql_injection,
    input_validator,
    form_validator,
    request_validator,
)
from typing import List, Dict, Any, Optional

from sentinel_web.internal.auth import get_current_user
from sentinel_web.models.users import User
from sentinel_web.drone_adapter.adapter import get_drone_adapter
from sentinel_web.drone_adapter.drone_client import DroneInfo

router = APIRouter()

@router.get("/", response_model=List[DroneInfo])
async def list_drones(current_user: User = Depends(get_current_user)):
    """
    List all available drones.
    """
    adapter = get_drone_adapter()
    drones = await adapter.get_drone_client().list_drones()
    return drones

@router.get("/{drone_id}", response_model=DroneInfo)
async def get_drone(drone_id: str, current_user: User = Depends(get_current_user)):
    """
    Get information about a specific drone.
    """
    adapter = get_drone_adapter()
    drone = await adapter.get_drone_client().get_drone(drone_id)
    
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
    adapter = get_drone_adapter()
    status_data = await adapter.get_drone_client().get_drone_status(drone_id)
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
    adapter = get_drone_adapter()
    
    command = command_data.get("command")
    parameters = command_data.get("parameters", {})
    
    if not command:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Command is required"
        )
    
    result = await adapter.get_drone_client().send_command(drone_id, command, parameters)
    return result
