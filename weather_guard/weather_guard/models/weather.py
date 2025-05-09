"""
Weather data models for Weather Guard service.

This module provides data models for weather data, forecasts,
and weather-related settings.
"""

from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional, Any, Union
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator


class WeatherProvider(str, Enum):
    """Weather data providers."""
    OPEN_METEO = "open-meteo"
    METEO_SHIELD = "meteo-shield"
    CUSTOM = "custom"


class WeatherCondition(str, Enum):
    """Weather condition types."""
    CLEAR = "clear"
    PARTLY_CLOUDY = "partly_cloudy"
    CLOUDY = "cloudy"
    OVERCAST = "overcast"
    FOG = "fog"
    LIGHT_RAIN = "light_rain"
    RAIN = "rain"
    HEAVY_RAIN = "heavy_rain"
    THUNDERSTORM = "thunderstorm"
    SNOW = "snow"
    SLEET = "sleet"
    HAIL = "hail"
    WINDY = "windy"
    UNKNOWN = "unknown"


class WeatherSeverity(str, Enum):
    """Weather severity levels for flight operations."""
    NONE = "none"  # No impact on operations
    LOW = "low"  # Minor impact, proceed with caution
    MEDIUM = "medium"  # Significant impact, consider alternatives
    HIGH = "high"  # Severe impact, operations not recommended
    EXTREME = "extreme"  # Extreme conditions, operations prohibited


class WeatherData(BaseModel):
    """Current weather data for a location."""
    id: UUID = Field(default_factory=uuid4)
    latitude: float
    longitude: float
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    provider: WeatherProvider
    condition: WeatherCondition
    temperature: float  # Celsius
    feels_like: Optional[float] = None  # Celsius
    humidity: Optional[float] = None  # Percentage
    pressure: Optional[float] = None  # hPa
    wind_speed: float  # m/s
    wind_direction: Optional[float] = None  # Degrees
    wind_gust: Optional[float] = None  # m/s
    visibility: Optional[float] = None  # km
    cloud_cover: Optional[float] = None  # Percentage
    precipitation: float  # mm/h
    uv_index: Optional[float] = None
    
    @field_validator("wind_speed", "precipitation")
    @classmethod
    def validate_non_negative(cls, v: float) -> float:
        """Validate that values are non-negative."""
        if v < 0:
            return 0.0
        return v


class WeatherForecast(BaseModel):
    """Weather forecast for a specific time."""
    id: UUID = Field(default_factory=uuid4)
    latitude: float
    longitude: float
    timestamp: datetime
    forecast_time: datetime
    provider: WeatherProvider
    condition: WeatherCondition
    temperature: float  # Celsius
    wind_speed: float  # m/s
    wind_direction: Optional[float] = None  # Degrees
    precipitation: float  # mm/h
    humidity: Optional[float] = None  # Percentage
    cloud_cover: Optional[float] = None  # Percentage


class WeatherAlert(BaseModel):
    """Weather alert for a location."""
    id: UUID = Field(default_factory=uuid4)
    latitude: float
    longitude: float
    start_time: datetime
    end_time: datetime
    provider: WeatherProvider
    severity: WeatherSeverity
    title: str
    description: str
    recommendation: Optional[str] = None


class MissionWeatherCheck(BaseModel):
    """Weather check for a mission."""
    id: UUID = Field(default_factory=uuid4)
    mission_id: Optional[str] = None
    latitude: float
    longitude: float
    check_time: datetime = Field(default_factory=datetime.utcnow)
    start_time: datetime
    end_time: Optional[datetime] = None
    is_flyable: bool
    severity: WeatherSeverity = WeatherSeverity.NONE
    current_weather: WeatherData
    forecast: List[WeatherForecast] = []
    alerts: List[WeatherAlert] = []
    limitations: List[Dict[str, Any]] = []
    recommendations: List[str] = []


class WeatherServiceStatus(BaseModel):
    """Status of the weather service."""
    status: str
    version: str
    providers: List[WeatherProvider]
    last_update: datetime
    cache_status: Dict[str, Any]
    error_count: int = 0
    uptime: float  # seconds


# Request and response models
class CurrentWeatherRequest(BaseModel):
    """Request model for current weather data."""
    latitude: float
    longitude: float
    provider: Optional[WeatherProvider] = None


class ForecastRequest(BaseModel):
    """Request model for weather forecast."""
    latitude: float
    longitude: float
    hours: int = 24
    provider: Optional[WeatherProvider] = None


class MissionWeatherCheckRequest(BaseModel):
    """Request model for mission weather check."""
    mission_id: Optional[str] = None
    latitude: float
    longitude: float
    start_time: datetime
    end_time: Optional[datetime] = None
    provider: Optional[WeatherProvider] = None


class WeatherResponse(BaseModel):
    """Base response model for weather data."""
    success: bool
    data: Optional[Union[WeatherData, List[WeatherForecast], MissionWeatherCheck, WeatherServiceStatus]] = None
    error: Optional[str] = None
