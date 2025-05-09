"""
Configuration module for Weather Guard service.

This module provides configuration settings for the Weather Guard service,
including API endpoints, Redis settings, and weather thresholds.
"""

import os
from typing import Any, Dict, List, Optional, Union

from pydantic import AnyHttpUrl, Field, RedisDsn, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Configuration settings for the Weather Guard service."""

    # API settings
    API_V1_STR: str = "/api"
    PROJECT_NAME: str = "Weather Guard"
    
    # CORS settings
    BACKEND_CORS_ORIGINS: List[str] = ["*"]
    
    # Service settings
    HOST: str = "0.0.0.0"
    PORT: int = 8090
    LOG_LEVEL: str = "INFO"
    
    # Weather settings
    DEFAULT_PROVIDER: str = "open-meteo"
    UPDATE_INTERVAL: int = 30  # minutes
    CACHE_DURATION: int = 30  # minutes
    WIND_THRESHOLD: float = 9.0  # m/s
    RAIN_THRESHOLD: float = 0.5  # mm/h
    
    # Open-Meteo settings
    OPEN_METEO_API_URL: str = "https://api.open-meteo.com/v1/forecast"
    OPEN_METEO_TIMEOUT: int = 10  # seconds
    
    # Redis settings
    REDIS_URL: Optional[RedisDsn] = "redis://localhost:6379/0"
    REDIS_PREFIX: str = "weather_guard:"
    REDIS_TTL: int = 1800  # seconds (30 minutes)
    
    # MQTT settings (for MeteoShield)
    MQTT_ENABLED: bool = False
    MQTT_BROKER: str = "localhost"
    MQTT_PORT: int = 1883
    MQTT_USERNAME: Optional[str] = None
    MQTT_PASSWORD: Optional[str] = None
    MQTT_TOPIC_PREFIX: str = "meteo_shield"
    MQTT_QOS: int = 1
    
    # Sentinel API integration
    SENTINEL_API_URL: Optional[AnyHttpUrl] = None
    SENTINEL_API_TOKEN: Optional[str] = None
    
    # Security
    SECRET_KEY: str = Field(default="changeme", env="SECRET_KEY")
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 60
    
    @field_validator("BACKEND_CORS_ORIGINS")
    def assemble_cors_origins(cls, v: Union[str, List[str]]) -> Union[List[str], str]:
        """Validate CORS origins."""
        if isinstance(v, str) and not v.startswith("["):
            return [i.strip() for i in v.split(",")]
        elif isinstance(v, (list, str)):
            return v
        raise ValueError(v)
    
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=True,
    )


# Create settings instance
settings = Settings()
