"""
Command API routes for Voice & Gesture Co-Pilot.

This module provides API endpoints for command processing and execution.
"""

import os
import time
from typing import Dict, Any, List, Optional
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status, Request, Body
from fastapi.responses import JSONResponse

from voice_gesture_copilot.core.security import get_current_user
from voice_gesture_copilot.services.command_service import CommandService
from voice_gesture_copilot.services.drone_service import DroneService

router = APIRouter()

def get_command_service(request: Request) -> CommandService:
    """Get the command service from the request state."""
    return request.app.state.command_service

def get_drone_service(request: Request) -> DroneService:
    """Get the drone service from the request state."""
    return request.app.state.drone_service

@router.post("/execute")
async def execute_command(
    request: Request,
    command_data: Dict[str, Any] = Body(...),
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Execute a drone command.
    
    Args:
        request: FastAPI request object
        command_data: Command data
        user: Current authenticated user
        
    Returns:
        Command execution results
    """
    # Get services
    drone_service = get_drone_service(request)
    
    try:
        # Extract command data
        drone_id = command_data.get("drone_id", "")
        command = command_data.get("command", "")
        parameters = command_data.get("parameters", {})
        
        # Validate command data
        if not drone_id:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Drone ID is required",
            )
        
        if not command:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Command is required",
            )
        
        # Send command to drone
        result = await drone_service.send_command(drone_id, command, parameters)
        
        return result
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error executing command: {str(e)}",
        )

@router.get("/history")
async def get_command_history(
    request: Request,
    limit: int = 100,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get command execution history.
    
    Args:
        request: FastAPI request object
        limit: Maximum number of history items to return
        user: Current authenticated user
        
    Returns:
        Command execution history
    """
    # Get services
    command_service = get_command_service(request)
    
    try:
        # Get command history
        history = command_service.command_history
        
        # Limit results
        history = history[-limit:]
        
        return {"history": history}
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting command history: {str(e)}",
        )

@router.get("/drone-history")
async def get_drone_command_history(
    request: Request,
    limit: int = 100,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get drone command execution history.
    
    Args:
        request: FastAPI request object
        limit: Maximum number of history items to return
        user: Current authenticated user
        
    Returns:
        Drone command execution history
    """
    # Get services
    drone_service = get_drone_service(request)
    
    try:
        # Get command history
        history = drone_service.command_history
        
        # Limit results
        history = history[-limit:]
        
        return {"history": history}
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting drone command history: {str(e)}",
        )

@router.get("/drones")
async def get_drones(
    request: Request,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get a list of available drones.
    
    Args:
        request: FastAPI request object
        user: Current authenticated user
        
    Returns:
        List of drone information
    """
    # Get services
    drone_service = get_drone_service(request)
    
    try:
        # Get drones
        drones = await drone_service.get_drones()
        
        return {"drones": drones}
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting drones: {str(e)}",
        )

@router.get("/drones/{drone_id}")
async def get_drone(
    request: Request,
    drone_id: str,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get information about a specific drone.
    
    Args:
        request: FastAPI request object
        drone_id: ID of the drone
        user: Current authenticated user
        
    Returns:
        Drone information
    """
    # Get services
    drone_service = get_drone_service(request)
    
    try:
        # Get drone
        drone = await drone_service.get_drone(drone_id)
        
        if not drone:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Drone {drone_id} not found",
            )
        
        return drone
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting drone {drone_id}: {str(e)}",
        )

@router.get("/drones/{drone_id}/telemetry")
async def get_drone_telemetry(
    request: Request,
    drone_id: str,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get telemetry data for a drone.
    
    Args:
        request: FastAPI request object
        drone_id: ID of the drone
        user: Current authenticated user
        
    Returns:
        Telemetry data
    """
    # Get services
    drone_service = get_drone_service(request)
    
    try:
        # Get telemetry
        telemetry = await drone_service.get_telemetry(drone_id)
        
        return telemetry
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting telemetry for drone {drone_id}: {str(e)}",
        )

@router.get("/status")
async def get_command_service_status(
    request: Request,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get command service status.
    
    Args:
        request: FastAPI request object
        user: Current authenticated user
        
    Returns:
        Command service status
    """
    # Get services
    command_service = get_command_service(request)
    
    try:
        # Get status
        status_info = await command_service.status()
        
        return status_info
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting command service status: {str(e)}",
        )

@router.get("/drone-service-status")
async def get_drone_service_status(
    request: Request,
    user: Dict[str, Any] = Depends(get_current_user),
):
    """
    Get drone service status.
    
    Args:
        request: FastAPI request object
        user: Current authenticated user
        
    Returns:
        Drone service status
    """
    # Get services
    drone_service = get_drone_service(request)
    
    try:
        # Get status
        status_info = await drone_service.status()
        
        return status_info
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting drone service status: {str(e)}",
        )
