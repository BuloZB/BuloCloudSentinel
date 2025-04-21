"""
Weather models for Bulo.Cloud Sentinel.

This module provides data models for weather data, forecasts,
and weather-related settings.
"""

from enum import Enum
from typing import List, Dict, Any, Optional, Union
from pydantic import BaseModel, Field, validator
import uuid
from datetime import datetime, timedelta

class WeatherProvider(str, Enum):
    """Weather data providers."""
    OPENWEATHERMAP = "openweathermap"
    WEATHERAPI = "weatherapi"
    AERISWEATHER = "aerisweather"
    TOMORROW = "tomorrow"
    VISUALCROSSING = "visualcrossing"
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
    DUST = "dust"
    SMOKE = "smoke"
    TORNADO = "tornado"
    HURRICANE = "hurricane"
    UNKNOWN = "unknown"

class WeatherSeverity(str, Enum):
    """Weather severity levels."""
    NONE = "none"
    LOW = "low"
    MODERATE = "moderate"
    HIGH = "high"
    EXTREME = "extreme"

class WeatherData(BaseModel):
    """Current weather data for a location."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    latitude: float
    longitude: float
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    provider: WeatherProvider
    condition: WeatherCondition
    temperature: float  # Celsius
    feels_like: Optional[float] = None  # Celsius
    humidity: Optional[float] = None  # Percentage
    pressure: Optional[float] = None  # hPa
    wind_speed: Optional[float] = None  # m/s
    wind_direction: Optional[float] = None  # Degrees
    wind_gust: Optional[float] = None  # m/s
    visibility: Optional[float] = None  # km
    cloud_cover: Optional[float] = None  # Percentage
    precipitation: Optional[float] = None  # mm
    uv_index: Optional[float] = None
    air_quality_index: Optional[float] = None
    severity: WeatherSeverity = WeatherSeverity.NONE
    raw_data: Optional[Dict[str, Any]] = None

class WeatherForecast(BaseModel):
    """Weather forecast for a location."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    latitude: float
    longitude: float
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    provider: WeatherProvider
    forecast_time: datetime
    condition: WeatherCondition
    temperature: float  # Celsius
    feels_like: Optional[float] = None  # Celsius
    humidity: Optional[float] = None  # Percentage
    pressure: Optional[float] = None  # hPa
    wind_speed: Optional[float] = None  # m/s
    wind_direction: Optional[float] = None  # Degrees
    wind_gust: Optional[float] = None  # m/s
    visibility: Optional[float] = None  # km
    cloud_cover: Optional[float] = None  # Percentage
    precipitation_probability: Optional[float] = None  # Percentage
    precipitation: Optional[float] = None  # mm
    uv_index: Optional[float] = None
    severity: WeatherSeverity = WeatherSeverity.NONE
    raw_data: Optional[Dict[str, Any]] = None

class WeatherAlert(BaseModel):
    """Weather alert for a location."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    latitude: float
    longitude: float
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    provider: WeatherProvider
    alert_type: str
    title: str
    description: str
    severity: WeatherSeverity
    start_time: datetime
    end_time: datetime
    affected_area: Optional[Dict[str, Any]] = None  # GeoJSON-like structure
    source: Optional[str] = None
    raw_data: Optional[Dict[str, Any]] = None

class WeatherProviderSettings(BaseModel):
    """Settings for a weather data provider."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    provider: WeatherProvider
    name: str
    description: Optional[str] = None
    api_key: Optional[str] = None
    api_url: Optional[str] = None
    enabled: bool = True
    update_interval: int = 30  # minutes
    last_update: Optional[datetime] = None
    settings: Dict[str, Any] = Field(default_factory=dict)

class WeatherSettings(BaseModel):
    """Global weather settings."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    default_provider: WeatherProvider = WeatherProvider.OPENWEATHERMAP
    update_interval: int = 30  # minutes
    cache_duration: int = 60  # minutes
    auto_refresh: bool = True
    providers: List[WeatherProviderSettings] = Field(default_factory=list)
    max_forecast_days: int = 7
    units: str = "metric"  # metric, imperial
    wind_speed_threshold: float = 10.0  # m/s
    visibility_threshold: float = 5.0  # km
    precipitation_threshold: float = 5.0  # mm
    temperature_min_threshold: float = -10.0  # Celsius
    temperature_max_threshold: float = 40.0  # Celsius

class WeatherDataResponse(WeatherData):
    """Response model for weather data."""
    pass

class WeatherForecastResponse(WeatherForecast):
    """Response model for weather forecast."""
    pass

class WeatherAlertResponse(WeatherAlert):
    """Response model for weather alert."""
    pass

class WeatherProviderSettingsResponse(WeatherProviderSettings):
    """Response model for weather provider settings."""
    # Hide API key in responses
    api_key: Optional[str] = None

class WeatherSettingsResponse(WeatherSettings):
    """Response model for weather settings."""
    # Include provider settings without API keys
    providers: List[WeatherProviderSettingsResponse] = Field(default_factory=list)

class WeatherDataRequest(BaseModel):
    """Request model for weather data."""
    latitude: float
    longitude: float
    provider: Optional[WeatherProvider] = None

class WeatherForecastRequest(BaseModel):
    """Request model for weather forecast."""
    latitude: float
    longitude: float
    days: Optional[int] = 1
    provider: Optional[WeatherProvider] = None

class WeatherAlertRequest(BaseModel):
    """Request model for weather alerts."""
    latitude: float
    longitude: float
    provider: Optional[WeatherProvider] = None

class MissionWeatherCheckRequest(BaseModel):
    """Request model for checking weather conditions for a mission."""
    mission_id: Optional[str] = None
    waypoints: Optional[List[Dict[str, Any]]] = None
    scheduled_time: Optional[datetime] = None

class WeatherCheckResult(BaseModel):
    """Result of a weather check for a mission."""
    valid: bool
    severity: WeatherSeverity = WeatherSeverity.NONE
    warnings: List[str] = Field(default_factory=list)
    weather_data: Optional[List[WeatherData]] = None
    forecasts: Optional[List[WeatherForecast]] = None
    alerts: Optional[List[WeatherAlert]] = None
