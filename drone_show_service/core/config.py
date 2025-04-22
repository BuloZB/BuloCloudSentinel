"""
Configuration for the Drone Show microservice.

This module provides configuration settings for the Drone Show microservice,
loaded from environment variables.
"""

import os
from typing import List, Optional, Union
from pydantic import BaseSettings, AnyHttpUrl, PostgresDsn, RedisDsn, validator


class Settings(BaseSettings):
    """
    Settings for the Drone Show microservice.
    
    These settings are loaded from environment variables.
    """
    
    # API settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Drone Show Microservice"
    
    # CORS settings
    CORS_ORIGINS: List[str] = ["*"]
    
    # Security settings
    SECRET_KEY: str = os.getenv("SECRET_KEY", "supersecretkey")
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 60 * 24 * 8  # 8 days
    
    # Database settings
    DATABASE_URL: PostgresDsn = os.getenv("DATABASE_URL", "postgresql://postgres:postgres@db:5432/drone_show")
    
    # Redis settings
    REDIS_URL: Optional[RedisDsn] = os.getenv("REDIS_URL", "redis://redis:6379/0")
    
    # MinIO settings
    MINIO_URL: str = os.getenv("MINIO_URL", "minio:9000")
    MINIO_ACCESS_KEY: str = os.getenv("MINIO_ACCESS_KEY", "minioadmin")
    MINIO_SECRET_KEY: str = os.getenv("MINIO_SECRET_KEY", "minioadmin")
    MINIO_BUCKET: str = os.getenv("MINIO_BUCKET", "drone-show")
    
    # RTMP server settings
    RTMP_SERVER: str = os.getenv("RTMP_SERVER", "rtmp://rtmp-server:1935")
    
    # Bulo.Cloud Sentinel integration
    SENTINEL_API_URL: str = os.getenv("SENTINEL_API_URL", "http://bulocloud-sentinel-api:8000")
    SENTINEL_API_TOKEN: Optional[str] = os.getenv("SENTINEL_API_TOKEN", "")
    
    # Logging settings
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")
    
    # WebSocket settings
    WS_PING_INTERVAL: int = 30  # seconds
    
    # Drone show settings
    MAX_DRONES: int = 200  # Maximum number of drones in a show
    MIN_DRONE_SPACING: float = 2.0  # Minimum distance between drones in meters
    DEFAULT_TAKEOFF_ALTITUDE: float = 10.0  # Default takeoff altitude in meters
    DEFAULT_LANDING_ALTITUDE: float = 1.0  # Default landing altitude in meters
    
    # Time synchronization settings
    TIME_SYNC_INTERVAL: int = 5  # seconds
    TIME_SYNC_TOLERANCE: float = 0.1  # seconds
    
    # LED control settings
    LED_UPDATE_RATE: int = 10  # Hz
    
    # Simulation settings
    SIMULATION_UPDATE_RATE: int = 30  # Hz
    
    class Config:
        """Pydantic config."""
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = True


# Create settings instance
settings = Settings()
