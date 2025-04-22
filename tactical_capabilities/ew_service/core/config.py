"""
Configuration settings for the EW service.
"""

import os
from typing import List, Optional
from pydantic import PostgresDsn, field_validator
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    """Settings for the EW service."""
    
    # API settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Electronic Warfare Service"
    
    # CORS settings
    CORS_ORIGINS: List[str] = ["http://localhost:3000", "http://localhost:8080"]
    
    # Database settings
    POSTGRES_HOST: str = os.getenv("POSTGRES_HOST", "localhost")
    POSTGRES_PORT: str = os.getenv("POSTGRES_PORT", "5432")
    POSTGRES_USER: str = os.getenv("POSTGRES_USER", "postgres")
    POSTGRES_PASSWORD: str = os.getenv("POSTGRES_PASSWORD", "postgres")
    POSTGRES_DB: str = os.getenv("POSTGRES_DB", "ew_service")
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
    JWT_SECRET: str = os.getenv("JWT_SECRET", "supersecretkey")
    JWT_ALGORITHM: str = "HS256"
    JWT_EXPIRATION_MINUTES: int = 30
    
    # RabbitMQ settings
    RABBITMQ_HOST: str = os.getenv("RABBITMQ_HOST", "localhost")
    RABBITMQ_PORT: int = int(os.getenv("RABBITMQ_PORT", "5672"))
    RABBITMQ_USERNAME: str = os.getenv("RABBITMQ_USERNAME", "guest")
    RABBITMQ_PASSWORD: str = os.getenv("RABBITMQ_PASSWORD", "guest")
    RABBITMQ_EXCHANGE: str = os.getenv("RABBITMQ_EXCHANGE", "bulo.sentinel.ew")
    
    # Redis settings
    REDIS_HOST: str = os.getenv("REDIS_HOST", "localhost")
    REDIS_PORT: int = int(os.getenv("REDIS_PORT", "6379"))
    REDIS_PASSWORD: str = os.getenv("REDIS_PASSWORD", "")
    REDIS_DB: int = int(os.getenv("REDIS_DB", "0"))
    
    # MinIO settings
    MINIO_ENDPOINT: str = os.getenv("MINIO_ENDPOINT", "localhost:9000")
    MINIO_ACCESS_KEY: str = os.getenv("MINIO_ACCESS_KEY", "minioadmin")
    MINIO_SECRET_KEY: str = os.getenv("MINIO_SECRET_KEY", "minioadmin")
    MINIO_BUCKET_NAME: str = os.getenv("MINIO_BUCKET_NAME", "ew")
    MINIO_SECURE: bool = os.getenv("MINIO_SECURE", "False").lower() == "true"
    
    # Platform settings
    PLATFORM_POLLING_INTERVAL: float = float(os.getenv("PLATFORM_POLLING_INTERVAL", "1.0"))
    
    # Attack settings
    ATTACK_MAX_DURATION: int = int(os.getenv("ATTACK_MAX_DURATION", "3600"))  # seconds
    ATTACK_POWER_LIMIT: float = float(os.getenv("ATTACK_POWER_LIMIT", "100.0"))  # Watts
    
    # Protection settings
    PROTECTION_MAX_DURATION: int = int(os.getenv("PROTECTION_MAX_DURATION", "86400"))  # seconds
    
    # Support settings
    SUPPORT_MAX_DURATION: int = int(os.getenv("SUPPORT_MAX_DURATION", "86400"))  # seconds
    
    # Spectrum settings
    SPECTRUM_MAX_SCAN_RANGE: float = float(os.getenv("SPECTRUM_MAX_SCAN_RANGE", "6e9"))  # Hz
    SPECTRUM_MIN_RESOLUTION: float = float(os.getenv("SPECTRUM_MIN_RESOLUTION", "1e3"))  # Hz
    
    # Threat settings
    THREAT_DETECTION_CONFIDENCE_THRESHOLD: float = float(os.getenv("THREAT_DETECTION_CONFIDENCE_THRESHOLD", "0.7"))
    
    # Countermeasure settings
    COUNTERMEASURE_MAX_DURATION: int = int(os.getenv("COUNTERMEASURE_MAX_DURATION", "3600"))  # seconds
    
    # Waveform settings
    WAVEFORM_MAX_SIZE: int = int(os.getenv("WAVEFORM_MAX_SIZE", "100"))  # MB
    
    # Security settings
    RATE_LIMIT_REQUESTS: int = int(os.getenv("RATE_LIMIT_REQUESTS", "100"))
    RATE_LIMIT_WINDOW_SECONDS: int = int(os.getenv("RATE_LIMIT_WINDOW_SECONDS", "60"))
    
    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
