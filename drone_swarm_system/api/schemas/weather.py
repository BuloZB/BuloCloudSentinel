"""
Weather schemas for the Drone Swarm System.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from enum import Enum

class WeatherParameter(str, Enum):
    """Weather parameter enumeration."""
    TEMPERATURE = "temperature"
    HUMIDITY = "humidity"
    PRESSURE = "pressure"
    WIND_SPEED = "wind_speed"
    WIND_DIRECTION = "wind_direction"
    WIND_GUST = "wind_gust"
    PRECIPITATION = "precipitation"
    PRECIPITATION_PROBABILITY = "precipitation_probability"
    CLOUD_COVER = "cloud_cover"
    VISIBILITY = "visibility"
    UV_INDEX = "uv_index"
    FEELS_LIKE = "feels_like"
    DEW_POINT = "dew_point"

class WeatherAlertType(str, Enum):
    """Weather alert type enumeration."""
    WIND = "wind"
    RAIN = "rain"
    SNOW = "snow"
    THUNDERSTORM = "thunderstorm"
    FOG = "fog"
    EXTREME_TEMPERATURE = "extreme_temperature"
    TORNADO = "tornado"
    HURRICANE = "hurricane"
    FLOOD = "flood"
    OTHER = "other"

class WeatherAlertSeverity(str, Enum):
    """Weather alert severity enumeration."""
    MINOR = "minor"
    MODERATE = "moderate"
    SEVERE = "severe"
    EXTREME = "extreme"

class WeatherSourceType(str, Enum):
    """Weather source type enumeration."""
    OPENWEATHERMAP = "openweathermap"
    WEATHERAPI = "weatherapi"
    ACCUWEATHER = "accuweather"
    DARKSKY = "darksky"
    METEOMATICS = "meteomatics"
    WEATHERBIT = "weatherbit"
    NOAA = "noaa"
    LOCAL_STATION = "local_station"
    CUSTOM = "custom"

class WeatherData(BaseModel):
    """Weather data model."""
    latitude: float
    longitude: float
    timestamp: datetime
    source: str
    temperature: Optional[float] = None  # Celsius
    humidity: Optional[float] = None  # Percent
    pressure: Optional[float] = None  # hPa
    wind_speed: Optional[float] = None  # m/s
    wind_direction: Optional[float] = None  # degrees
    wind_gust: Optional[float] = None  # m/s
    precipitation: Optional[float] = None  # mm
    precipitation_probability: Optional[float] = None  # Percent
    cloud_cover: Optional[float] = None  # Percent
    visibility: Optional[float] = None  # meters
    uv_index: Optional[float] = None
    feels_like: Optional[float] = None  # Celsius
    dew_point: Optional[float] = None  # Celsius
    weather_code: Optional[str] = None
    weather_description: Optional[str] = None
    icon: Optional[str] = None

class WeatherForecast(BaseModel):
    """Weather forecast model."""
    latitude: float
    longitude: float
    source: str
    generated_at: datetime
    forecasts: List[WeatherData]

class WeatherAlert(BaseModel):
    """Weather alert model."""
    id: str
    alert_type: WeatherAlertType
    severity: WeatherAlertSeverity
    title: str
    description: str
    source: str
    area: Optional[Dict[str, Any]] = None
    start_time: datetime
    end_time: datetime
    parameters: Optional[Dict[str, Any]] = None
    url: Optional[str] = None

class WeatherSource(BaseModel):
    """Weather data source model."""
    id: str
    name: str
    source_type: WeatherSourceType
    api_key: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None
    enabled: bool = True
    priority: int = 1  # higher number = higher priority
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

class WeatherAlertConfig(BaseModel):
    """Weather alert configuration model."""
    alert_type: WeatherAlertType
    threshold: float
    enabled: bool = True
    parameters: Optional[Dict[str, Any]] = None

class WeatherCheckResult(BaseModel):
    """Result of a weather check for a mission."""
    mission_id: str
    timestamp: datetime
    weather_data: WeatherData
    forecast: Optional[List[WeatherData]] = None
    alerts: List[WeatherAlert] = []
    is_flyable: bool
    limitations: List[Dict[str, Any]] = []
    recommendations: List[str] = []
