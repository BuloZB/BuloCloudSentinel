"""
SentinelWeb Backend - Drones Endpoints

This module provides endpoints for drone management.
"""

from typing import Any, List, Optional

from fastapi import APIRouter, Depends, HTTPException, status, Request
from pydantic import BaseModel

from backend.core.auth import get_current_user, User, has_role
from backend.services.sentinel_client import SentinelClient

router = APIRouter()

# Pydantic models
class DroneBase(BaseModel):
    """Base drone model."""
    name: str
    model: str
    serial_number: str

class DroneCreate(DroneBase):
    """Drone creation model."""
    pass

class DroneUpdate(BaseModel):
    """Drone update model."""
    name: Optional[str] = None
    model: Optional[str] = None
    serial_number: Optional[str] = None
    is_active: Optional[bool] = None

class DroneResponse(DroneBase):
    """Drone response model."""
    id: str
    status: str
    battery_level: Optional[float] = None
    location: Optional[dict] = None
    is_active: bool

class DroneCommand(BaseModel):
    """Drone command model."""
    command: str
    parameters: Optional[dict] = None

@router.get("/", response_model=List[DroneResponse])
async def get_drones(
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Get all drones.
    
    Args:
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        List of drones
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Get drones from BuloCloudSentinel
        drones = await sentinel_client.get_drones()
        return drones
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error getting drones from BuloCloudSentinel: {str(e)}"
        )

@router.get("/{drone_id}", response_model=DroneResponse)
async def get_drone(
    drone_id: str,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Get a drone by ID.
    
    Args:
        drone_id: Drone ID
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Drone information
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Get drone from BuloCloudSentinel
        drone = await sentinel_client.get_drone(drone_id)
        return drone
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error getting drone from BuloCloudSentinel: {str(e)}"
        )

@router.post("/", response_model=DroneResponse, dependencies=[Depends(has_role(["admin", "operator"]))])
async def create_drone(
    drone: DroneCreate,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Create a new drone.
    
    Args:
        drone: Drone data
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Created drone
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Create drone in BuloCloudSentinel
        # This is a placeholder - the actual implementation would depend on BuloCloudSentinel's API
        created_drone = await sentinel_client._request(
            method="POST",
            endpoint="/api/device-inventory/drones",
            data=drone.dict()
        )
        return created_drone
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error creating drone in BuloCloudSentinel: {str(e)}"
        )

@router.put("/{drone_id}", response_model=DroneResponse, dependencies=[Depends(has_role(["admin", "operator"]))])
async def update_drone(
    drone_id: str,
    drone: DroneUpdate,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Update a drone.
    
    Args:
        drone_id: Drone ID
        drone: Drone data
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Updated drone
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Update drone in BuloCloudSentinel
        # This is a placeholder - the actual implementation would depend on BuloCloudSentinel's API
        updated_drone = await sentinel_client._request(
            method="PUT",
            endpoint=f"/api/device-inventory/drones/{drone_id}",
            data=drone.dict(exclude_unset=True)
        )
        return updated_drone
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error updating drone in BuloCloudSentinel: {str(e)}"
        )

@router.delete("/{drone_id}", dependencies=[Depends(has_role(["admin"]))])
async def delete_drone(
    drone_id: str,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Delete a drone.
    
    Args:
        drone_id: Drone ID
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Deletion confirmation
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Delete drone in BuloCloudSentinel
        # This is a placeholder - the actual implementation would depend on BuloCloudSentinel's API
        await sentinel_client._request(
            method="DELETE",
            endpoint=f"/api/device-inventory/drones/{drone_id}"
        )
        return {"detail": "Drone deleted"}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error deleting drone in BuloCloudSentinel: {str(e)}"
        )

@router.post("/{drone_id}/command", dependencies=[Depends(has_role(["admin", "operator"]))])
async def send_drone_command(
    drone_id: str,
    command: DroneCommand,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Send a command to a drone.
    
    Args:
        drone_id: Drone ID
        command: Command data
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Command response
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Send command to drone via BuloCloudSentinel
        # This is a placeholder - the actual implementation would depend on BuloCloudSentinel's API
        response = await sentinel_client._request(
            method="POST",
            endpoint=f"/api/device-inventory/drones/{drone_id}/command",
            data=command.dict()
        )
        return response
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error sending command to drone: {str(e)}"
        )

@router.get("/{drone_id}/telemetry")
async def get_drone_telemetry(
    drone_id: str,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Get drone telemetry.
    
    Args:
        drone_id: Drone ID
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Drone telemetry
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Get telemetry from BuloCloudSentinel
        telemetry = await sentinel_client.get_telemetry(drone_id)
        return telemetry
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error getting drone telemetry: {str(e)}"
        )

@router.get("/{drone_id}/battery")
async def get_drone_battery(
    drone_id: str,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Get drone battery status.
    
    Args:
        drone_id: Drone ID
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Drone battery status
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Get battery status from BuloCloudSentinel
        battery = await sentinel_client.get_battery_status(drone_id)
        return battery
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error getting drone battery status: {str(e)}"
        )
