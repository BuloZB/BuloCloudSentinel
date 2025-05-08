"""
Configuration settings for the SentinelBeacon module.
"""

import os
from typing import List, Optional, Dict, Any
from pydantic import PostgresDsn, field_validator
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    """Settings for the SentinelBeacon module."""

    # API settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "SentinelBeacon"

    # CORS settings
    CORS_ORIGINS: List[str] = ["http://localhost:3000", "http://localhost:8080"]

    # Database settings
    POSTGRES_HOST: str = os.getenv("POSTGRES_HOST", "localhost")
    POSTGRES_PORT: str = os.getenv("POSTGRES_PORT", "5432")
    POSTGRES_USER: str = os.getenv("POSTGRES_USER", "postgres")
    POSTGRES_PASSWORD: str = os.getenv("POSTGRES_PASSWORD", "postgres")
    POSTGRES_DB: str = os.getenv("POSTGRES_DB", "sentinel_beacon")
    DATABASE_URL: Optional[PostgresDsn] = None

    @field_validator("DATABASE_URL", mode="before")
    def assemble_db_url(cls, v: Optional[str], values) -> str:
        if v:
            return v
        return PostgresDsn.build(
            scheme="postgresql+asyncpg",
            username=values.data.get("POSTGRES_USER"),
            password=values.data.get("POSTGRES_PASSWORD"),
            host=values.data.get("POSTGRES_HOST"),
            port=values.data.get("POSTGRES_PORT"),
            path=f"{values.data.get('POSTGRES_DB') or ''}",
        )

    # JWT settings
    JWT_SECRET: str = os.getenv("JWT_SECRET", "")
    JWT_ALGORITHM: str = "HS256"
    JWT_EXPIRATION_MINUTES: int = 30

    @field_validator("JWT_SECRET", mode="before")
    def validate_jwt_secret(cls, v: str) -> str:
        if not v:
            if os.getenv("ENVIRONMENT", "development").lower() == "production":
                raise ValueError("JWT_SECRET environment variable must be set in production")
            else:
                import secrets
                import logging
                v = secrets.token_hex(32)
                logging.warning("Generated random JWT secret key for development. This will change on restart!")
        return v

    # RabbitMQ settings
    RABBITMQ_HOST: str = os.getenv("RABBITMQ_HOST", "localhost")
    RABBITMQ_PORT: int = int(os.getenv("RABBITMQ_PORT", "5672"))
    RABBITMQ_USERNAME: str = os.getenv("RABBITMQ_USERNAME", "guest")
    RABBITMQ_PASSWORD: str = os.getenv("RABBITMQ_PASSWORD", "guest")
    RABBITMQ_EXCHANGE: str = os.getenv("RABBITMQ_EXCHANGE", "bulo.sentinel.beacon")

    # Redis settings
    REDIS_HOST: str = os.getenv("REDIS_HOST", "localhost")
    REDIS_PORT: int = int(os.getenv("REDIS_PORT", "6379"))
    REDIS_PASSWORD: str = os.getenv("REDIS_PASSWORD", "")
    REDIS_DB: int = int(os.getenv("REDIS_DB", "0"))

    # Meshtastic settings
    MESHTASTIC_DEVICE: str = os.getenv("MESHTASTIC_DEVICE", "/dev/ttyUSB0")
    MESHTASTIC_BAUDRATE: int = int(os.getenv("MESHTASTIC_BAUDRATE", "115200"))
    MESHTASTIC_REGION: str = os.getenv("MESHTASTIC_REGION", "US")  # US, EU433, EU868, CN, JP, ANZ, KR, TW
    MESHTASTIC_MODEM_CONFIG: str = os.getenv("MESHTASTIC_MODEM_CONFIG", "LongFast")  # LongFast, LongSlow, VLongSlow, MediumSlow, MediumFast, ShortSlow, ShortFast
    MESHTASTIC_TX_POWER: int = int(os.getenv("MESHTASTIC_TX_POWER", "20"))  # dBm
    MESHTASTIC_NODE_NAME: str = os.getenv("MESHTASTIC_NODE_NAME", "SentinelBeacon")
    MESHTASTIC_POSITION_BROADCAST_SECS: int = int(os.getenv("MESHTASTIC_POSITION_BROADCAST_SECS", "30"))
    MESHTASTIC_BLUETOOTH_ENABLED: bool = os.getenv("MESHTASTIC_BLUETOOTH_ENABLED", "True").lower() == "true"
    MESHTASTIC_CHANNEL_NAME: str = os.getenv("MESHTASTIC_CHANNEL_NAME", "SentinelNet")
    MESHTASTIC_CHANNEL_PSK: Optional[str] = os.getenv("MESHTASTIC_CHANNEL_PSK", None)  # Optional encryption key

    # Drone interface settings
    DRONE_INTERFACE_TYPE: str = os.getenv("DRONE_INTERFACE_TYPE", "mavlink")  # mavlink, serial, none
    DRONE_INTERFACE_PORT: str = os.getenv("DRONE_INTERFACE_PORT", "/dev/ttyACM0")
    DRONE_INTERFACE_BAUDRATE: int = int(os.getenv("DRONE_INTERFACE_BAUDRATE", "57600"))
    DRONE_INTERFACE_SYSTEM_ID: int = int(os.getenv("DRONE_INTERFACE_SYSTEM_ID", "1"))
    DRONE_INTERFACE_COMPONENT_ID: int = int(os.getenv("DRONE_INTERFACE_COMPONENT_ID", "1"))

    # Channel settings
    DEFAULT_CHANNELS: List[Dict[str, Any]] = [
        {
            "name": "beacon",
            "description": "SentinelBeacon primary channel",
            "enabled": True,
            "role": "PRIMARY",
            "psk": None  # No encryption by default
        },
        {
            "name": "emergency",
            "description": "Emergency communication channel",
            "enabled": True,
            "role": "SECONDARY",
            "psk": None
        }
    ]

    # Beacon settings
    BEACON_MODE: str = os.getenv("BEACON_MODE", "autonomous")  # autonomous, manual, scheduled
    BEACON_ALTITUDE: float = float(os.getenv("BEACON_ALTITUDE", "50.0"))  # meters
    BEACON_HOVER_TIME: int = int(os.getenv("BEACON_HOVER_TIME", "1800"))  # seconds (30 minutes)
    BEACON_RETURN_HOME_BATTERY: int = int(os.getenv("BEACON_RETURN_HOME_BATTERY", "30"))  # percent
    BEACON_MAX_DISTANCE: float = float(os.getenv("BEACON_MAX_DISTANCE", "500.0"))  # meters from home
    BEACON_MOVEMENT_PATTERN: str = os.getenv("BEACON_MOVEMENT_PATTERN", "hover")  # hover, circle, grid

    # Message settings
    MESSAGE_RETENTION_DAYS: int = int(os.getenv("MESSAGE_RETENTION_DAYS", "7"))
    MESSAGE_MAX_SIZE: int = int(os.getenv("MESSAGE_MAX_SIZE", "240"))  # bytes

    # Telemetry settings
    TELEMETRY_BROADCAST_INTERVAL: int = int(os.getenv("TELEMETRY_BROADCAST_INTERVAL", "60"))  # seconds
    TELEMETRY_RETENTION_DAYS: int = int(os.getenv("TELEMETRY_RETENTION_DAYS", "7"))

    # Position settings
    POSITION_BROADCAST_INTERVAL: int = int(os.getenv("POSITION_BROADCAST_INTERVAL", "5"))  # seconds
    POSITION_RETENTION_DAYS: int = int(os.getenv("POSITION_RETENTION_DAYS", "7"))

    # Security settings
    RATE_LIMIT_REQUESTS: int = int(os.getenv("RATE_LIMIT_REQUESTS", "100"))
    RATE_LIMIT_WINDOW_SECONDS: int = int(os.getenv("RATE_LIMIT_WINDOW_SECONDS", "60"))

    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
