"""
Weather API endpoints for Weather Guard service.

This module provides API endpoints for weather data, forecasts, and mission weather checks.
"""

import logging
from datetime import datetime
from typing import List, Optional

from fastapi import APIRouter, Depends, HTTPException, Query

from weather_guard.models.weather import (
    CurrentWeatherRequest,
    ForecastRequest,
    MissionWeatherCheck,
    MissionWeatherCheckRequest,
    WeatherData,
    WeatherForecast,
    WeatherProvider,
    WeatherResponse,
    WeatherServiceStatus,
)
from weather_guard.services.weather import WeatherService

logger = logging.getLogger(__name__)

router = APIRouter()


def get_weather_service() -> WeatherService:
    """Get the weather service instance."""
    return WeatherService()


@router.get("/current", response_model=WeatherResponse)
async def get_current_weather(
    latitude: float = Query(..., description="Latitude of the location"),
    longitude: float = Query(..., description="Longitude of the location"),
    provider: Optional[WeatherProvider] = Query(None, description="Weather data provider"),
    weather_service: WeatherService = Depends(get_weather_service),
) -> WeatherResponse:
    """Get current weather data for a location."""
    try:
        weather_data = await weather_service.get_current_weather(latitude, longitude, provider)
        return WeatherResponse(success=True, data=weather_data)
    except Exception as e:
        logger.error(f"Error getting current weather: {str(e)}")
        return WeatherResponse(success=False, error=f"Error getting current weather: {str(e)}")


@router.post("/current", response_model=WeatherResponse)
async def post_current_weather(
    request: CurrentWeatherRequest,
    weather_service: WeatherService = Depends(get_weather_service),
) -> WeatherResponse:
    """Get current weather data for a location (POST method)."""
    try:
        weather_data = await weather_service.get_current_weather(
            request.latitude, request.longitude, request.provider
        )
        return WeatherResponse(success=True, data=weather_data)
    except Exception as e:
        logger.error(f"Error getting current weather: {str(e)}")
        return WeatherResponse(success=False, error=f"Error getting current weather: {str(e)}")


@router.get("/forecast", response_model=WeatherResponse)
async def get_forecast(
    latitude: float = Query(..., description="Latitude of the location"),
    longitude: float = Query(..., description="Longitude of the location"),
    hours: int = Query(24, description="Number of hours to forecast"),
    provider: Optional[WeatherProvider] = Query(None, description="Weather data provider"),
    weather_service: WeatherService = Depends(get_weather_service),
) -> WeatherResponse:
    """Get weather forecast for a location."""
    try:
        forecast_data = await weather_service.get_forecast(latitude, longitude, hours, provider)
        return WeatherResponse(success=True, data=forecast_data)
    except Exception as e:
        logger.error(f"Error getting forecast: {str(e)}")
        return WeatherResponse(success=False, error=f"Error getting forecast: {str(e)}")


@router.post("/forecast", response_model=WeatherResponse)
async def post_forecast(
    request: ForecastRequest,
    weather_service: WeatherService = Depends(get_weather_service),
) -> WeatherResponse:
    """Get weather forecast for a location (POST method)."""
    try:
        forecast_data = await weather_service.get_forecast(
            request.latitude, request.longitude, request.hours, request.provider
        )
        return WeatherResponse(success=True, data=forecast_data)
    except Exception as e:
        logger.error(f"Error getting forecast: {str(e)}")
        return WeatherResponse(success=False, error=f"Error getting forecast: {str(e)}")


@router.get("/check-mission", response_model=WeatherResponse)
async def check_mission_weather(
    latitude: float = Query(..., description="Latitude of the mission location"),
    longitude: float = Query(..., description="Longitude of the mission location"),
    start_time: datetime = Query(..., description="Mission start time"),
    end_time: Optional[datetime] = Query(None, description="Mission end time"),
    mission_id: Optional[str] = Query(None, description="Mission ID"),
    provider: Optional[WeatherProvider] = Query(None, description="Weather data provider"),
    weather_service: WeatherService = Depends(get_weather_service),
) -> WeatherResponse:
    """Check if weather conditions are suitable for a mission."""
    try:
        check_result = await weather_service.check_mission_weather(
            latitude, longitude, start_time, end_time, mission_id, provider
        )
        return WeatherResponse(success=True, data=check_result)
    except Exception as e:
        logger.error(f"Error checking mission weather: {str(e)}")
        return WeatherResponse(success=False, error=f"Error checking mission weather: {str(e)}")


@router.post("/check-mission", response_model=WeatherResponse)
async def post_check_mission_weather(
    request: MissionWeatherCheckRequest,
    weather_service: WeatherService = Depends(get_weather_service),
) -> WeatherResponse:
    """Check if weather conditions are suitable for a mission (POST method)."""
    try:
        check_result = await weather_service.check_mission_weather(
            request.latitude,
            request.longitude,
            request.start_time,
            request.end_time,
            request.mission_id,
            request.provider,
        )
        return WeatherResponse(success=True, data=check_result)
    except Exception as e:
        logger.error(f"Error checking mission weather: {str(e)}")
        return WeatherResponse(success=False, error=f"Error checking mission weather: {str(e)}")


@router.get("/status", response_model=WeatherResponse)
async def get_service_status(
    weather_service: WeatherService = Depends(get_weather_service),
) -> WeatherResponse:
    """Get the status of the weather service."""
    try:
        status = await weather_service.get_service_status()
        return WeatherResponse(success=True, data=status)
    except Exception as e:
        logger.error(f"Error getting service status: {str(e)}")
        return WeatherResponse(success=False, error=f"Error getting service status: {str(e)}")
