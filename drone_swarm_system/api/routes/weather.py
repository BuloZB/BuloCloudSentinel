"""
Weather API routes for the Drone Swarm System.

This module provides endpoints for weather monitoring and integration.
"""

from fastapi import APIRouter, Depends, HTTPException, Request
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from datetime import datetime

# Import local modules
from safety.weather_monitor import WeatherMonitor
from api.schemas.weather import (
    WeatherData,
    WeatherForecast,
    WeatherSource,
    WeatherAlert
)

router = APIRouter()

# Models
class WeatherSourceConfig(BaseModel):
    """Request model for configuring a weather data source."""
    source_type: str
    api_key: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None
    enabled: bool = True

class WeatherAlertConfig(BaseModel):
    """Request model for configuring weather alerts."""
    alert_type: str
    threshold: float
    enabled: bool = True

# Helper functions
def get_weather_monitor(request: Request) -> WeatherMonitor:
    """Get the weather monitor from the app state."""
    return request.app.state.weather_monitor

# Routes
@router.get("/current")
async def get_current_weather(
    latitude: Optional[float] = None,
    longitude: Optional[float] = None,
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Get current weather data for a location."""
    try:
        weather = await weather_monitor.get_current_weather(latitude, longitude)
        return weather
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting current weather: {str(e)}")

@router.get("/forecast")
async def get_weather_forecast(
    latitude: Optional[float] = None,
    longitude: Optional[float] = None,
    hours: int = 24,
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Get weather forecast for a location."""
    try:
        forecast = await weather_monitor.get_forecast(latitude, longitude, hours)
        return forecast
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting weather forecast: {str(e)}")

@router.get("/alerts")
async def get_weather_alerts(
    latitude: Optional[float] = None,
    longitude: Optional[float] = None,
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Get active weather alerts for a location."""
    try:
        alerts = await weather_monitor.get_alerts(latitude, longitude)
        return alerts
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting weather alerts: {str(e)}")

@router.get("/sources")
async def get_weather_sources(
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Get all configured weather data sources."""
    try:
        sources = await weather_monitor.get_sources()
        return sources
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting weather sources: {str(e)}")

@router.post("/sources")
async def add_weather_source(
    source: WeatherSourceConfig,
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Add a new weather data source."""
    try:
        new_source = await weather_monitor.add_source(
            source.source_type,
            source.api_key,
            source.parameters,
            source.enabled
        )
        return new_source
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error adding weather source: {str(e)}")

@router.delete("/sources/{source_id}")
async def remove_weather_source(
    source_id: str,
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Remove a weather data source."""
    try:
        success = await weather_monitor.remove_source(source_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Weather source {source_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error removing weather source: {str(e)}")

@router.get("/config/alerts")
async def get_alert_config(
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Get weather alert configuration."""
    try:
        config = await weather_monitor.get_alert_config()
        return config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting alert configuration: {str(e)}")

@router.post("/config/alerts")
async def update_alert_config(
    config: List[WeatherAlertConfig],
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Update weather alert configuration."""
    try:
        updated_config = await weather_monitor.update_alert_config(
            [
                {
                    "alert_type": alert.alert_type,
                    "threshold": alert.threshold,
                    "enabled": alert.enabled
                }
                for alert in config
            ]
        )
        return updated_config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating alert configuration: {str(e)}")

@router.get("/check-mission/{mission_id}")
async def check_weather_for_mission(
    mission_id: str,
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Check if weather conditions are suitable for a mission."""
    try:
        result = await weather_monitor.check_mission(mission_id)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error checking weather for mission: {str(e)}")

@router.get("/history")
async def get_weather_history(
    latitude: float,
    longitude: float,
    start_date: datetime,
    end_date: datetime,
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Get historical weather data for a location."""
    try:
        history = await weather_monitor.get_history(latitude, longitude, start_date, end_date)
        return history
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting weather history: {str(e)}")

@router.get("/parameters")
async def get_weather_parameters(
    weather_monitor: WeatherMonitor = Depends(get_weather_monitor)
):
    """Get all available weather parameters."""
    try:
        parameters = await weather_monitor.get_parameters()
        return parameters
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting weather parameters: {str(e)}")
