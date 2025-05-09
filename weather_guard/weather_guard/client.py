"""
Weather Guard client for integration with other services.

This module provides a client for the Weather Guard service, which can be used
by other services to access weather data and check mission weather conditions.
"""

import logging
from datetime import datetime
from typing import Dict, List, Optional, Any, Union

import httpx
from pydantic import ValidationError

from weather_guard.models.weather import (
    WeatherData,
    WeatherForecast,
    MissionWeatherCheck,
    WeatherProvider,
    WeatherResponse,
)

logger = logging.getLogger(__name__)


class WeatherGuardClient:
    """Client for the Weather Guard service."""
    
    def __init__(
        self,
        base_url: str = "http://weather-guard:8090/api",
        timeout: int = 10,
    ):
        """Initialize the Weather Guard client.
        
        Args:
            base_url: Base URL of the Weather Guard API
            timeout: Timeout for API requests in seconds
        """
        self.base_url = base_url
        self.timeout = timeout
        self.client = httpx.AsyncClient(timeout=self.timeout)
    
    async def close(self) -> None:
        """Close the HTTP client."""
        await self.client.aclose()
    
    async def get_current_weather(
        self,
        latitude: float,
        longitude: float,
        provider: Optional[WeatherProvider] = None,
    ) -> WeatherData:
        """Get current weather data for a location.
        
        Args:
            latitude: Latitude of the location
            longitude: Longitude of the location
            provider: Weather data provider
            
        Returns:
            Current weather data
            
        Raises:
            httpx.HTTPError: If the API request fails
            ValueError: If the API response is invalid
        """
        params = {
            "latitude": latitude,
            "longitude": longitude,
        }
        
        if provider:
            params["provider"] = provider
        
        try:
            response = await self.client.get(f"{self.base_url}/weather/current", params=params)
            response.raise_for_status()
            data = response.json()
            
            if not data.get("success", False):
                raise ValueError(data.get("error", "Unknown error"))
            
            weather_data = WeatherData.model_validate(data["data"])
            return weather_data
        
        except httpx.HTTPError as e:
            logger.error(f"HTTP error fetching current weather: {str(e)}")
            raise
        except (KeyError, ValueError, ValidationError) as e:
            logger.error(f"Error parsing weather data: {str(e)}")
            raise ValueError(f"Invalid weather data: {str(e)}")
    
    async def get_forecast(
        self,
        latitude: float,
        longitude: float,
        hours: int = 24,
        provider: Optional[WeatherProvider] = None,
    ) -> List[WeatherForecast]:
        """Get weather forecast for a location.
        
        Args:
            latitude: Latitude of the location
            longitude: Longitude of the location
            hours: Number of hours to forecast
            provider: Weather data provider
            
        Returns:
            List of weather forecasts
            
        Raises:
            httpx.HTTPError: If the API request fails
            ValueError: If the API response is invalid
        """
        params = {
            "latitude": latitude,
            "longitude": longitude,
            "hours": hours,
        }
        
        if provider:
            params["provider"] = provider
        
        try:
            response = await self.client.get(f"{self.base_url}/weather/forecast", params=params)
            response.raise_for_status()
            data = response.json()
            
            if not data.get("success", False):
                raise ValueError(data.get("error", "Unknown error"))
            
            forecast_data = [WeatherForecast.model_validate(item) for item in data["data"]]
            return forecast_data
        
        except httpx.HTTPError as e:
            logger.error(f"HTTP error fetching forecast: {str(e)}")
            raise
        except (KeyError, ValueError, ValidationError) as e:
            logger.error(f"Error parsing forecast data: {str(e)}")
            raise ValueError(f"Invalid forecast data: {str(e)}")
    
    async def check_mission_weather(
        self,
        latitude: float,
        longitude: float,
        start_time: datetime,
        end_time: Optional[datetime] = None,
        mission_id: Optional[str] = None,
        provider: Optional[WeatherProvider] = None,
    ) -> MissionWeatherCheck:
        """Check if weather conditions are suitable for a mission.
        
        Args:
            latitude: Latitude of the mission location
            longitude: Longitude of the mission location
            start_time: Mission start time
            end_time: Mission end time (optional)
            mission_id: Mission ID (optional)
            provider: Weather data provider
            
        Returns:
            Mission weather check result
            
        Raises:
            httpx.HTTPError: If the API request fails
            ValueError: If the API response is invalid
        """
        params = {
            "latitude": latitude,
            "longitude": longitude,
            "start_time": start_time.isoformat(),
        }
        
        if end_time:
            params["end_time"] = end_time.isoformat()
        
        if mission_id:
            params["mission_id"] = mission_id
        
        if provider:
            params["provider"] = provider
        
        try:
            response = await self.client.get(f"{self.base_url}/weather/check-mission", params=params)
            response.raise_for_status()
            data = response.json()
            
            if not data.get("success", False):
                raise ValueError(data.get("error", "Unknown error"))
            
            check_result = MissionWeatherCheck.model_validate(data["data"])
            return check_result
        
        except httpx.HTTPError as e:
            logger.error(f"HTTP error checking mission weather: {str(e)}")
            raise
        except (KeyError, ValueError, ValidationError) as e:
            logger.error(f"Error parsing mission weather check data: {str(e)}")
            raise ValueError(f"Invalid mission weather check data: {str(e)}")
    
    async def get_service_status(self) -> Dict[str, Any]:
        """Get the status of the Weather Guard service.
        
        Returns:
            Service status
            
        Raises:
            httpx.HTTPError: If the API request fails
            ValueError: If the API response is invalid
        """
        try:
            response = await self.client.get(f"{self.base_url}/weather/status")
            response.raise_for_status()
            data = response.json()
            
            if not data.get("success", False):
                raise ValueError(data.get("error", "Unknown error"))
            
            return data["data"]
        
        except httpx.HTTPError as e:
            logger.error(f"HTTP error getting service status: {str(e)}")
            raise
        except (KeyError, ValueError) as e:
            logger.error(f"Error parsing service status data: {str(e)}")
            raise ValueError(f"Invalid service status data: {str(e)}")
