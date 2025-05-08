"""
API Endpoints for Dock Driver Microservice

This module defines the API endpoints for the Dock Driver microservice.
"""

from fastapi import APIRouter, Depends, HTTPException, Path, Query
from fastapi.security import OAuth2PasswordBearer
from typing import Dict, Any, List, Optional
import logging

from dock_driver.models.schemas import (
    DockStatusResponse, DockTelemetryResponse, 
    DockListResponse, DockItem, ErrorResponse,
    SuccessResponse
)
from dock_driver.services.dock_manager import DockManager
from dock_driver.services.auth import verify_token

logger = logging.getLogger(__name__)

# Create router
router = APIRouter()

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


# Dependency to get the dock manager
async def get_dock_manager():
    """Get the dock manager instance."""
    return DockManager.get_instance()


# Dependency to verify authentication
async def get_current_user(token: str = Depends(oauth2_scheme)):
    """Verify the authentication token and return the user."""
    user = await verify_token(token)
    if not user:
        raise HTTPException(
            status_code=401,
            detail="Invalid authentication credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return user


@router.get(
    "/docks", 
    response_model=DockListResponse,
    summary="Get all docks",
    description="Get a list of all configured docks."
)
async def get_docks(
    current_user: Dict[str, Any] = Depends(get_current_user),
    dock_manager: DockManager = Depends(get_dock_manager)
):
    """
    Get a list of all configured docks.
    
    Args:
        current_user: Current authenticated user.
        dock_manager: Dock manager instance.
        
    Returns:
        List of docks.
    """
    try:
        docks = dock_manager.get_all_docks()
        return DockListResponse(docks=docks)
    except Exception as e:
        logger.error(f"Error getting docks: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error getting docks: {str(e)}"
        )


@router.get(
    "/docks/{dock_type}/{dock_id}/status", 
    response_model=DockStatusResponse,
    summary="Get dock status",
    description="Get the current status of a specific dock."
)
async def get_dock_status(
    dock_type: str = Path(..., description="Type of the dock (dji, heisha, esp32)"),
    dock_id: str = Path(..., description="ID of the dock"),
    current_user: Dict[str, Any] = Depends(get_current_user),
    dock_manager: DockManager = Depends(get_dock_manager)
):
    """
    Get the current status of a specific dock.
    
    Args:
        dock_type: Type of the dock (dji, heisha, esp32).
        dock_id: ID of the dock.
        current_user: Current authenticated user.
        dock_manager: Dock manager instance.
        
    Returns:
        Dock status.
    """
    try:
        adapter = dock_manager.get_adapter(dock_type, dock_id)
        if not adapter:
            raise HTTPException(
                status_code=404,
                detail=f"Dock {dock_id} of type {dock_type} not found"
            )
        
        status = await adapter.get_status()
        return DockStatusResponse(**status)
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting dock status: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error getting dock status: {str(e)}"
        )


@router.get(
    "/docks/{dock_type}/{dock_id}/telemetry", 
    response_model=DockTelemetryResponse,
    summary="Get dock telemetry",
    description="Get telemetry data from a specific dock."
)
async def get_dock_telemetry(
    dock_type: str = Path(..., description="Type of the dock (dji, heisha, esp32)"),
    dock_id: str = Path(..., description="ID of the dock"),
    current_user: Dict[str, Any] = Depends(get_current_user),
    dock_manager: DockManager = Depends(get_dock_manager)
):
    """
    Get telemetry data from a specific dock.
    
    Args:
        dock_type: Type of the dock (dji, heisha, esp32).
        dock_id: ID of the dock.
        current_user: Current authenticated user.
        dock_manager: Dock manager instance.
        
    Returns:
        Dock telemetry.
    """
    try:
        adapter = dock_manager.get_adapter(dock_type, dock_id)
        if not adapter:
            raise HTTPException(
                status_code=404,
                detail=f"Dock {dock_id} of type {dock_type} not found"
            )
        
        telemetry = await adapter.get_telemetry()
        return DockTelemetryResponse(**telemetry)
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting dock telemetry: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error getting dock telemetry: {str(e)}"
        )


@router.post(
    "/docks/{dock_type}/{dock_id}/open", 
    response_model=SuccessResponse,
    summary="Open dock",
    description="Open a specific dock."
)
async def open_dock(
    dock_type: str = Path(..., description="Type of the dock (dji, heisha, esp32)"),
    dock_id: str = Path(..., description="ID of the dock"),
    current_user: Dict[str, Any] = Depends(get_current_user),
    dock_manager: DockManager = Depends(get_dock_manager)
):
    """
    Open a specific dock.
    
    Args:
        dock_type: Type of the dock (dji, heisha, esp32).
        dock_id: ID of the dock.
        current_user: Current authenticated user.
        dock_manager: Dock manager instance.
        
    Returns:
        Success response.
    """
    try:
        adapter = dock_manager.get_adapter(dock_type, dock_id)
        if not adapter:
            raise HTTPException(
                status_code=404,
                detail=f"Dock {dock_id} of type {dock_type} not found"
            )
        
        success = await adapter.open_dock()
        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to open dock {dock_id}"
            )
        
        return SuccessResponse(message=f"Dock {dock_id} opened successfully")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error opening dock: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error opening dock: {str(e)}"
        )


@router.post(
    "/docks/{dock_type}/{dock_id}/close", 
    response_model=SuccessResponse,
    summary="Close dock",
    description="Close a specific dock."
)
async def close_dock(
    dock_type: str = Path(..., description="Type of the dock (dji, heisha, esp32)"),
    dock_id: str = Path(..., description="ID of the dock"),
    current_user: Dict[str, Any] = Depends(get_current_user),
    dock_manager: DockManager = Depends(get_dock_manager)
):
    """
    Close a specific dock.
    
    Args:
        dock_type: Type of the dock (dji, heisha, esp32).
        dock_id: ID of the dock.
        current_user: Current authenticated user.
        dock_manager: Dock manager instance.
        
    Returns:
        Success response.
    """
    try:
        adapter = dock_manager.get_adapter(dock_type, dock_id)
        if not adapter:
            raise HTTPException(
                status_code=404,
                detail=f"Dock {dock_id} of type {dock_type} not found"
            )
        
        success = await adapter.close_dock()
        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to close dock {dock_id}"
            )
        
        return SuccessResponse(message=f"Dock {dock_id} closed successfully")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error closing dock: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error closing dock: {str(e)}"
        )


@router.post(
    "/docks/{dock_type}/{dock_id}/charge/start", 
    response_model=SuccessResponse,
    summary="Start charging",
    description="Start charging a drone at a specific dock."
)
async def start_charging(
    dock_type: str = Path(..., description="Type of the dock (dji, heisha, esp32)"),
    dock_id: str = Path(..., description="ID of the dock"),
    current_user: Dict[str, Any] = Depends(get_current_user),
    dock_manager: DockManager = Depends(get_dock_manager)
):
    """
    Start charging a drone at a specific dock.
    
    Args:
        dock_type: Type of the dock (dji, heisha, esp32).
        dock_id: ID of the dock.
        current_user: Current authenticated user.
        dock_manager: Dock manager instance.
        
    Returns:
        Success response.
    """
    try:
        adapter = dock_manager.get_adapter(dock_type, dock_id)
        if not adapter:
            raise HTTPException(
                status_code=404,
                detail=f"Dock {dock_id} of type {dock_type} not found"
            )
        
        success = await adapter.start_charging()
        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to start charging at dock {dock_id}"
            )
        
        return SuccessResponse(message=f"Charging started at dock {dock_id}")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error starting charging: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error starting charging: {str(e)}"
        )


@router.post(
    "/docks/{dock_type}/{dock_id}/charge/stop", 
    response_model=SuccessResponse,
    summary="Stop charging",
    description="Stop charging a drone at a specific dock."
)
async def stop_charging(
    dock_type: str = Path(..., description="Type of the dock (dji, heisha, esp32)"),
    dock_id: str = Path(..., description="ID of the dock"),
    current_user: Dict[str, Any] = Depends(get_current_user),
    dock_manager: DockManager = Depends(get_dock_manager)
):
    """
    Stop charging a drone at a specific dock.
    
    Args:
        dock_type: Type of the dock (dji, heisha, esp32).
        dock_id: ID of the dock.
        current_user: Current authenticated user.
        dock_manager: Dock manager instance.
        
    Returns:
        Success response.
    """
    try:
        adapter = dock_manager.get_adapter(dock_type, dock_id)
        if not adapter:
            raise HTTPException(
                status_code=404,
                detail=f"Dock {dock_id} of type {dock_type} not found"
            )
        
        success = await adapter.stop_charging()
        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to stop charging at dock {dock_id}"
            )
        
        return SuccessResponse(message=f"Charging stopped at dock {dock_id}")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error stopping charging: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error stopping charging: {str(e)}"
        )


# Heisha-specific endpoints
@router.post(
    "/docks/heisha/{dock_id}/fan/{speed}", 
    response_model=SuccessResponse,
    summary="Set fan speed",
    description="Set the fan speed for a Heisha Charging Pad."
)
async def set_fan_speed(
    dock_id: str = Path(..., description="ID of the dock"),
    speed: int = Path(..., description="Fan speed (0-100%)"),
    current_user: Dict[str, Any] = Depends(get_current_user),
    dock_manager: DockManager = Depends(get_dock_manager)
):
    """
    Set the fan speed for a Heisha Charging Pad.
    
    Args:
        dock_id: ID of the dock.
        speed: Fan speed (0-100%).
        current_user: Current authenticated user.
        dock_manager: Dock manager instance.
        
    Returns:
        Success response.
    """
    try:
        adapter = dock_manager.get_adapter("heisha", dock_id)
        if not adapter:
            raise HTTPException(
                status_code=404,
                detail=f"Heisha dock {dock_id} not found"
            )
        
        # Validate speed
        if speed < 0 or speed > 100:
            raise HTTPException(
                status_code=400,
                detail="Fan speed must be between 0 and 100"
            )
        
        success = await adapter.set_fan_speed(speed)
        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to set fan speed for dock {dock_id}"
            )
        
        return SuccessResponse(message=f"Fan speed set to {speed}% for dock {dock_id}")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error setting fan speed: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error setting fan speed: {str(e)}"
        )


@router.post(
    "/docks/heisha/{dock_id}/heater/{state}", 
    response_model=SuccessResponse,
    summary="Set heater state",
    description="Set the heater state for a Heisha Charging Pad."
)
async def set_heater_state(
    dock_id: str = Path(..., description="ID of the dock"),
    state: str = Path(..., description="Heater state (on/off)"),
    current_user: Dict[str, Any] = Depends(get_current_user),
    dock_manager: DockManager = Depends(get_dock_manager)
):
    """
    Set the heater state for a Heisha Charging Pad.
    
    Args:
        dock_id: ID of the dock.
        state: Heater state (on/off).
        current_user: Current authenticated user.
        dock_manager: Dock manager instance.
        
    Returns:
        Success response.
    """
    try:
        adapter = dock_manager.get_adapter("heisha", dock_id)
        if not adapter:
            raise HTTPException(
                status_code=404,
                detail=f"Heisha dock {dock_id} not found"
            )
        
        # Validate state
        if state.lower() not in ["on", "off"]:
            raise HTTPException(
                status_code=400,
                detail="Heater state must be 'on' or 'off'"
            )
        
        success = await adapter.set_heater_state(state.lower() == "on")
        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to set heater state for dock {dock_id}"
            )
        
        return SuccessResponse(message=f"Heater {state} for dock {dock_id}")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error setting heater state: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error setting heater state: {str(e)}"
        )


# ESP32-specific endpoints
@router.post(
    "/docks/esp32/{dock_id}/relay/{state}", 
    response_model=SuccessResponse,
    summary="Set relay state",
    description="Set the relay state for an ESP32-powered dock."
)
async def set_relay_state(
    dock_id: str = Path(..., description="ID of the dock"),
    state: str = Path(..., description="Relay state (on/off)"),
    current_user: Dict[str, Any] = Depends(get_current_user),
    dock_manager: DockManager = Depends(get_dock_manager)
):
    """
    Set the relay state for an ESP32-powered dock.
    
    Args:
        dock_id: ID of the dock.
        state: Relay state (on/off).
        current_user: Current authenticated user.
        dock_manager: Dock manager instance.
        
    Returns:
        Success response.
    """
    try:
        adapter = dock_manager.get_adapter("esp32", dock_id)
        if not adapter:
            raise HTTPException(
                status_code=404,
                detail=f"ESP32 dock {dock_id} not found"
            )
        
        # Validate state
        if state.lower() not in ["on", "off"]:
            raise HTTPException(
                status_code=400,
                detail="Relay state must be 'on' or 'off'"
            )
        
        success = await adapter.set_relay_state(state.lower() == "on")
        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to set relay state for dock {dock_id}"
            )
        
        return SuccessResponse(message=f"Relay {state} for dock {dock_id}")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error setting relay state: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error setting relay state: {str(e)}"
        )
