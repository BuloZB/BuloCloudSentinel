"""
Open-Meteo API client for Weather Guard service.

This module provides a client for the Open-Meteo API, which is used to fetch
weather data and forecasts.
"""

import logging
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any, Tuple

import httpx
from pydantic import ValidationError

from weather_guard.core.config import settings
from weather_guard.models.weather import (
    WeatherCondition,
    WeatherData,
    WeatherForecast,
    WeatherProvider,
)

logger = logging.getLogger(__name__)


class OpenMeteoClient:
    """Client for the Open-Meteo API."""

    def __init__(self, api_url: Optional[str] = None, timeout: int = 10):
        """Initialize the Open-Meteo API client.
        
        Args:
            api_url: URL of the Open-Meteo API
            timeout: Timeout for API requests in seconds
        """
        self.api_url = api_url or settings.OPEN_METEO_API_URL
        self.timeout = timeout or settings.OPEN_METEO_TIMEOUT
        self.client = httpx.AsyncClient(timeout=self.timeout)
    
    async def close(self) -> None:
        """Close the HTTP client."""
        await self.client.aclose()
    
    async def get_current_weather(self, latitude: float, longitude: float) -> WeatherData:
        """Get current weather data for a location.
        
        Args:
            latitude: Latitude of the location
            longitude: Longitude of the location
            
        Returns:
            Current weather data
            
        Raises:
            httpx.HTTPError: If the API request fails
            ValueError: If the API response is invalid
        """
        params = {
            "latitude": latitude,
            "longitude": longitude,
            "current": "temperature_2m,relative_humidity_2m,apparent_temperature,precipitation,rain,weather_code,cloud_cover,pressure_msl,wind_speed_10m,wind_direction_10m,wind_gusts_10m",
            "timezone": "UTC",
        }
        
        try:
            response = await self.client.get(self.api_url, params=params)
            response.raise_for_status()
            data = response.json()
            
            # Extract current weather data
            current = data.get("current", {})
            if not current:
                raise ValueError("No current weather data in response")
            
            # Map weather code to condition
            weather_code = current.get("weather_code", 0)
            condition = self._map_weather_code_to_condition(weather_code)
            
            # Create WeatherData object
            weather_data = WeatherData(
                latitude=latitude,
                longitude=longitude,
                timestamp=datetime.fromisoformat(current.get("time", datetime.utcnow().isoformat())),
                provider=WeatherProvider.OPEN_METEO,
                condition=condition,
                temperature=current.get("temperature_2m", 0.0),
                feels_like=current.get("apparent_temperature", None),
                humidity=current.get("relative_humidity_2m", None),
                pressure=current.get("pressure_msl", None),
                wind_speed=current.get("wind_speed_10m", 0.0),
                wind_direction=current.get("wind_direction_10m", None),
                wind_gust=current.get("wind_gusts_10m", None),
                cloud_cover=current.get("cloud_cover", None),
                precipitation=current.get("precipitation", 0.0) or current.get("rain", 0.0),
            )
            
            return weather_data
        
        except httpx.HTTPError as e:
            logger.error(f"HTTP error fetching current weather: {str(e)}")
            raise
        except (KeyError, ValueError, ValidationError) as e:
            logger.error(f"Error parsing weather data: {str(e)}")
            raise ValueError(f"Invalid weather data: {str(e)}")
    
    async def get_forecast(self, latitude: float, longitude: float, hours: int = 24) -> List[WeatherForecast]:
        """Get weather forecast for a location.
        
        Args:
            latitude: Latitude of the location
            longitude: Longitude of the location
            hours: Number of hours to forecast
            
        Returns:
            List of weather forecasts
            
        Raises:
            httpx.HTTPError: If the API request fails
            ValueError: If the API response is invalid
        """
        params = {
            "latitude": latitude,
            "longitude": longitude,
            "hourly": "temperature_2m,precipitation,rain,weather_code,cloud_cover,wind_speed_10m,wind_direction_10m,relative_humidity_2m",
            "forecast_hours": min(hours, 168),  # Max 7 days (168 hours)
            "timezone": "UTC",
        }
        
        try:
            response = await self.client.get(self.api_url, params=params)
            response.raise_for_status()
            data = response.json()
            
            # Extract hourly forecast data
            hourly = data.get("hourly", {})
            if not hourly:
                raise ValueError("No hourly forecast data in response")
            
            # Get time array
            time_array = hourly.get("time", [])
            if not time_array:
                raise ValueError("No time data in forecast")
            
            # Create forecast objects
            forecasts = []
            for i, time_str in enumerate(time_array):
                if i >= hours:
                    break
                
                # Map weather code to condition
                weather_code = hourly.get("weather_code", [])[i] if "weather_code" in hourly and i < len(hourly["weather_code"]) else 0
                condition = self._map_weather_code_to_condition(weather_code)
                
                # Create forecast object
                forecast = WeatherForecast(
                    latitude=latitude,
                    longitude=longitude,
                    timestamp=datetime.utcnow(),
                    forecast_time=datetime.fromisoformat(time_str),
                    provider=WeatherProvider.OPEN_METEO,
                    condition=condition,
                    temperature=hourly.get("temperature_2m", [])[i] if "temperature_2m" in hourly and i < len(hourly["temperature_2m"]) else 0.0,
                    wind_speed=hourly.get("wind_speed_10m", [])[i] if "wind_speed_10m" in hourly and i < len(hourly["wind_speed_10m"]) else 0.0,
                    wind_direction=hourly.get("wind_direction_10m", [])[i] if "wind_direction_10m" in hourly and i < len(hourly["wind_direction_10m"]) else None,
                    precipitation=hourly.get("precipitation", [])[i] if "precipitation" in hourly and i < len(hourly["precipitation"]) else 0.0,
                    humidity=hourly.get("relative_humidity_2m", [])[i] if "relative_humidity_2m" in hourly and i < len(hourly["relative_humidity_2m"]) else None,
                    cloud_cover=hourly.get("cloud_cover", [])[i] if "cloud_cover" in hourly and i < len(hourly["cloud_cover"]) else None,
                )
                
                forecasts.append(forecast)
            
            return forecasts
        
        except httpx.HTTPError as e:
            logger.error(f"HTTP error fetching forecast: {str(e)}")
            raise
        except (KeyError, ValueError, ValidationError) as e:
            logger.error(f"Error parsing forecast data: {str(e)}")
            raise ValueError(f"Invalid forecast data: {str(e)}")
    
    def _map_weather_code_to_condition(self, code: int) -> WeatherCondition:
        """Map Open-Meteo weather code to WeatherCondition.
        
        Args:
            code: Open-Meteo weather code
            
        Returns:
            Corresponding WeatherCondition
        """
        # WMO Weather interpretation codes (WW)
        # https://open-meteo.com/en/docs
        if code == 0:  # Clear sky
            return WeatherCondition.CLEAR
        elif code in [1, 2, 3]:  # Mainly clear, partly cloudy, and overcast
            return WeatherCondition.PARTLY_CLOUDY
        elif code == 45 or code == 48:  # Fog and depositing rime fog
            return WeatherCondition.FOG
        elif code in [51, 53, 55]:  # Drizzle: Light, moderate, and dense intensity
            return WeatherCondition.LIGHT_RAIN
        elif code in [56, 57]:  # Freezing Drizzle: Light and dense intensity
            return WeatherCondition.SLEET
        elif code in [61, 63, 65]:  # Rain: Slight, moderate and heavy intensity
            if code == 61:
                return WeatherCondition.LIGHT_RAIN
            elif code == 63:
                return WeatherCondition.RAIN
            else:
                return WeatherCondition.HEAVY_RAIN
        elif code in [66, 67]:  # Freezing Rain: Light and heavy intensity
            return WeatherCondition.SLEET
        elif code in [71, 73, 75]:  # Snow fall: Slight, moderate, and heavy intensity
            return WeatherCondition.SNOW
        elif code == 77:  # Snow grains
            return WeatherCondition.SNOW
        elif code in [80, 81, 82]:  # Rain showers: Slight, moderate, and violent
            if code == 80:
                return WeatherCondition.LIGHT_RAIN
            elif code == 81:
                return WeatherCondition.RAIN
            else:
                return WeatherCondition.HEAVY_RAIN
        elif code in [85, 86]:  # Snow showers slight and heavy
            return WeatherCondition.SNOW
        elif code in [95, 96, 99]:  # Thunderstorm: Slight or moderate, with hail
            return WeatherCondition.THUNDERSTORM
        else:
            return WeatherCondition.UNKNOWN
