"""
Weather alert API endpoints for Weather Guard service.

This module provides API endpoints for weather alerts and weather windows.
"""

import logging
from datetime import datetime
from typing import Dict, List, Optional, Any

from fastapi import APIRouter, Depends, HTTPException, Query, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from weather_guard.models.weather import WeatherAlert
from weather_guard.services.alerts import AlertService
from weather_guard.services.weather import WeatherService

logger = logging.getLogger(__name__)

router = APIRouter()


# Request and response models
class LocationRequest(BaseModel):
    """Request model for adding a location."""
    latitude: float
    longitude: float
    name: Optional[str] = None


class LocationResponse(BaseModel):
    """Response model for a location."""
    id: str
    latitude: float
    longitude: float
    name: str


class AlertResponse(BaseModel):
    """Response model for an alert."""
    alerts: List[WeatherAlert]


class WindowResponse(BaseModel):
    """Response model for a weather window."""
    windows: List[Dict[str, Any]]


class ApiResponse(BaseModel):
    """Generic API response model."""
    success: bool
    data: Optional[Any] = None
    error: Optional[str] = None


# Alert service instance
alert_service = None


def get_alert_service() -> AlertService:
    """Get the alert service instance."""
    global alert_service
    if alert_service is None:
        # Create alert service
        weather_service = WeatherService()
        alert_service = AlertService(weather_service)
    return alert_service


@router.post("/locations", response_model=ApiResponse)
async def add_location(
    request: LocationRequest,
    alert_service: AlertService = Depends(get_alert_service),
) -> ApiResponse:
    """Add a location to monitor."""
    try:
        location_id = alert_service.add_location(
            request.latitude, request.longitude, request.name
        )
        
        return ApiResponse(
            success=True,
            data={
                "id": location_id,
                "latitude": request.latitude,
                "longitude": request.longitude,
                "name": request.name or f"{request.latitude}, {request.longitude}",
            },
        )
    except Exception as e:
        logger.error(f"Error adding location: {str(e)}")
        return ApiResponse(success=False, error=f"Error adding location: {str(e)}")


@router.delete("/locations/{location_id}", response_model=ApiResponse)
async def remove_location(
    location_id: str,
    alert_service: AlertService = Depends(get_alert_service),
) -> ApiResponse:
    """Remove a location from monitoring."""
    try:
        success = alert_service.remove_location(location_id)
        
        if success:
            return ApiResponse(success=True)
        else:
            return ApiResponse(success=False, error=f"Location {location_id} not found")
    except Exception as e:
        logger.error(f"Error removing location: {str(e)}")
        return ApiResponse(success=False, error=f"Error removing location: {str(e)}")


@router.get("/locations", response_model=ApiResponse)
async def get_locations(
    alert_service: AlertService = Depends(get_alert_service),
) -> ApiResponse:
    """Get all monitored locations."""
    try:
        locations = alert_service.get_locations()
        return ApiResponse(success=True, data=locations)
    except Exception as e:
        logger.error(f"Error getting locations: {str(e)}")
        return ApiResponse(success=False, error=f"Error getting locations: {str(e)}")


@router.get("/alerts", response_model=ApiResponse)
async def get_alerts(
    location_id: Optional[str] = None,
    alert_service: AlertService = Depends(get_alert_service),
) -> ApiResponse:
    """Get active weather alerts."""
    try:
        alerts = alert_service.get_active_alerts()
        
        # Filter by location if provided
        if location_id:
            alerts = [a for a in alerts if a.id.startswith(f"{location_id}:")]
        
        return ApiResponse(success=True, data=alerts)
    except Exception as e:
        logger.error(f"Error getting alerts: {str(e)}")
        return ApiResponse(success=False, error=f"Error getting alerts: {str(e)}")


@router.get("/windows", response_model=ApiResponse)
async def get_windows(
    location_id: Optional[str] = None,
    alert_service: AlertService = Depends(get_alert_service),
) -> ApiResponse:
    """Get weather windows."""
    try:
        windows = alert_service.get_weather_windows()
        
        # Filter by location if provided
        if location_id:
            windows = [w for w in windows if w["location_id"] == location_id]
        
        # Convert datetime objects to strings
        for window in windows:
            window["start_time"] = window["start_time"].isoformat()
            window["end_time"] = window["end_time"].isoformat()
            window["forecast"] = [f.model_dump() for f in window["forecast"]]
        
        return ApiResponse(success=True, data=windows)
    except Exception as e:
        logger.error(f"Error getting windows: {str(e)}")
        return ApiResponse(success=False, error=f"Error getting windows: {str(e)}")


@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time alerts and weather windows."""
    await websocket.accept()
    
    # Get alert service
    alert_service = get_alert_service()
    
    # Define notification callback
    async def send_notification(data: Dict[str, Any]) -> None:
        try:
            await websocket.send_json(data)
        except Exception as e:
            logger.error(f"Error sending WebSocket notification: {str(e)}")
    
    # Add callback
    alert_service.add_notification_callback(send_notification)
    
    try:
        # Start alert service if not running
        if not alert_service.running:
            await alert_service.start()
        
        # Send initial data
        await websocket.send_json({
            "type": "init",
            "alerts": [a.model_dump() for a in alert_service.get_active_alerts()],
            "windows": [
                {
                    "id": w["id"],
                    "location_id": w["location_id"],
                    "location_name": w["location_name"],
                    "latitude": w["latitude"],
                    "longitude": w["longitude"],
                    "start_time": w["start_time"].isoformat(),
                    "end_time": w["end_time"].isoformat(),
                    "duration_hours": w["duration_hours"],
                }
                for w in alert_service.get_weather_windows()
            ],
        })
        
        # Wait for messages
        while True:
            data = await websocket.receive_text()
            # Process commands if needed
    
    except WebSocketDisconnect:
        # Remove callback
        alert_service.remove_notification_callback(send_notification)
    
    except Exception as e:
        logger.error(f"WebSocket error: {str(e)}")
        # Remove callback
        alert_service.remove_notification_callback(send_notification)
