"""
Telemetry API router for SentinelWeb.
"""

from fastapi import APIRouter, Depends, HTTPException, status, WebSocket, WebSocketDisconnect

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
from datetime import datetime

from sentinel_web.internal.auth import get_current_user
from sentinel_web.models.users import User
from sentinel_web.drone_adapter.adapter import get_drone_adapter
from sentinel_web.drone_adapter.telemetry_client import TelemetryData

router = APIRouter()

@router.get("/{drone_id}")
async def get_current_telemetry(drone_id: str, current_user: User = Depends(get_current_user)):
    """
    Get current telemetry for a drone.
    """
    adapter = get_drone_adapter()
    telemetry = await adapter.get_telemetry_client().get_current_telemetry(drone_id)
    
    if not telemetry:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Telemetry for drone {drone_id} not found"
        )
    
    return telemetry

@router.get("/{drone_id}/history")
async def get_telemetry_history(
    drone_id: str,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    interval: str = "1m",
    current_user: User = Depends(get_current_user)
):
    """
    Get historical telemetry for a drone.
    """
    adapter = get_drone_adapter()
    history = await adapter.get_telemetry_client().get_telemetry_history(
        drone_id, start_time, end_time, interval
    )
    
    return {"data": history}

@router.websocket("/ws/{drone_id}")
async def websocket_telemetry(websocket: WebSocket, drone_id: str):
    """
    WebSocket endpoint for real-time telemetry.
    """
    await websocket.accept()
    
    adapter = get_drone_adapter()
    
    try:
        async for telemetry in adapter.get_telemetry_client().get_telemetry_stream(drone_id):
            await websocket.send_json(telemetry.dict())
    except WebSocketDisconnect:
        pass
    except Exception as e:
        await websocket.close(code=1000, reason=str(e))

@router.get("/battery/{drone_id}")
async def get_battery_status(drone_id: str, current_user: User = Depends(get_current_user)):
    """
    Get battery status for a drone.
    """
    adapter = get_drone_adapter()
    battery = await adapter.get_telemetry_client().get_battery_status(drone_id)
    
    if "error" in battery:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=battery["error"]
        )
    
    return battery

@router.get("/battery/{drone_id}/history")
async def get_battery_history(
    drone_id: str,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    interval: str = "1m",
    current_user: User = Depends(get_current_user)
):
    """
    Get battery history for a drone.
    """
    adapter = get_drone_adapter()
    history = await adapter.get_telemetry_client().get_battery_history(
        drone_id, start_time, end_time, interval
    )
    
    if "error" in history:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=history["error"]
        )
    
    return history
