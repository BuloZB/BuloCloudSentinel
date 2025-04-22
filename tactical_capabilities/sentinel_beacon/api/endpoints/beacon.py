"""
API endpoints for beacon management.
"""

from typing import List, Optional
from fastapi import APIRouter, Depends, HTTPException, Query, status, Request
from datetime import datetime

from api.schemas import BeaconConfig, BeaconStatus, BeaconMode, BeaconMovementPattern
from core.security import get_current_user, has_permission
from core.config import settings

router = APIRouter()

@router.get("/status", response_model=BeaconStatus)
async def get_beacon_status(
    request: Request,
    current_user = Depends(get_current_user)
):
    """
    Get the current status of the beacon.
    
    Args:
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Beacon status
    """
    # Get drone interface
    drone_interface = request.app.state.drone_interface
    
    # Check if connected
    if not drone_interface.connected:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Drone interface not connected"
        )
    
    # Get telemetry
    telemetry = await drone_interface.get_telemetry()
    
    # Create beacon status
    beacon_status = BeaconStatus(
        active=drone_interface.beacon_active,
        mode=drone_interface.beacon_mode,
        position=telemetry.get("position"),
        battery_level=telemetry.get("battery_level", 0.0),
        start_time=None,  # Not implemented yet
        elapsed_time=None  # Not implemented yet
    )
    
    return beacon_status

@router.get("/config", response_model=BeaconConfig)
async def get_beacon_config(
    current_user = Depends(get_current_user)
):
    """
    Get the current configuration of the beacon.
    
    Args:
        current_user: Current authenticated user
        
    Returns:
        Beacon configuration
    """
    # Create beacon config
    beacon_config = BeaconConfig(
        mode=settings.BEACON_MODE,
        altitude=settings.BEACON_ALTITUDE,
        hover_time=settings.BEACON_HOVER_TIME,
        return_home_battery=settings.BEACON_RETURN_HOME_BATTERY,
        max_distance=settings.BEACON_MAX_DISTANCE,
        movement_pattern=settings.BEACON_MOVEMENT_PATTERN
    )
    
    return beacon_config

@router.put("/config", response_model=BeaconConfig)
async def update_beacon_config(
    config: BeaconConfig,
    request: Request,
    current_user = Depends(has_permission("beacon:update"))
):
    """
    Update the configuration of the beacon.
    
    Args:
        config: Beacon configuration
        request: FastAPI request
        current_user: Current authenticated user with required permission
        
    Returns:
        Updated beacon configuration
    """
    # Get drone interface
    drone_interface = request.app.state.drone_interface
    
    # Check if connected
    if not drone_interface.connected:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Drone interface not connected"
        )
    
    # Update beacon mode
    await drone_interface.set_beacon_mode(config.mode)
    
    # Update settings
    settings.BEACON_MODE = config.mode
    settings.BEACON_ALTITUDE = config.altitude
    settings.BEACON_HOVER_TIME = config.hover_time
    settings.BEACON_RETURN_HOME_BATTERY = config.return_home_battery
    settings.BEACON_MAX_DISTANCE = config.max_distance
    settings.BEACON_MOVEMENT_PATTERN = config.movement_pattern
    
    return config

@router.post("/start", response_model=BeaconStatus)
async def start_beacon(
    request: Request,
    current_user = Depends(has_permission("beacon:control"))
):
    """
    Start the beacon operation.
    
    Args:
        request: FastAPI request
        current_user: Current authenticated user with required permission
        
    Returns:
        Beacon status
    """
    # Get drone interface
    drone_interface = request.app.state.drone_interface
    
    # Check if connected
    if not drone_interface.connected:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Drone interface not connected"
        )
    
    # Start beacon
    success = await drone_interface.start_beacon()
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to start beacon"
        )
    
    # Get telemetry
    telemetry = await drone_interface.get_telemetry()
    
    # Create beacon status
    beacon_status = BeaconStatus(
        active=drone_interface.beacon_active,
        mode=drone_interface.beacon_mode,
        position=telemetry.get("position"),
        battery_level=telemetry.get("battery_level", 0.0),
        start_time=datetime.utcnow(),
        elapsed_time=0
    )
    
    return beacon_status

@router.post("/stop", response_model=BeaconStatus)
async def stop_beacon(
    request: Request,
    current_user = Depends(has_permission("beacon:control"))
):
    """
    Stop the beacon operation.
    
    Args:
        request: FastAPI request
        current_user: Current authenticated user with required permission
        
    Returns:
        Beacon status
    """
    # Get drone interface
    drone_interface = request.app.state.drone_interface
    
    # Check if connected
    if not drone_interface.connected:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Drone interface not connected"
        )
    
    # Stop beacon
    success = await drone_interface.stop_beacon()
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to stop beacon"
        )
    
    # Get telemetry
    telemetry = await drone_interface.get_telemetry()
    
    # Create beacon status
    beacon_status = BeaconStatus(
        active=drone_interface.beacon_active,
        mode=drone_interface.beacon_mode,
        position=telemetry.get("position"),
        battery_level=telemetry.get("battery_level", 0.0),
        start_time=None,
        elapsed_time=None
    )
    
    return beacon_status

@router.post("/return-home")
async def return_home(
    request: Request,
    current_user = Depends(has_permission("beacon:control"))
):
    """
    Return the drone to home position.
    
    Args:
        request: FastAPI request
        current_user: Current authenticated user with required permission
        
    Returns:
        Success message
    """
    # Get drone interface
    drone_interface = request.app.state.drone_interface
    
    # Check if connected
    if not drone_interface.connected:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Drone interface not connected"
        )
    
    # Return home
    success = await drone_interface.return_home()
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to return home"
        )
    
    return {"message": "Returning to home position"}
