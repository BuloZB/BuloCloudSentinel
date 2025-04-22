"""
Configuration settings for the ISR service.
"""

import os
from typing import List, Optional
from pydantic import PostgresDsn, field_validator
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    """Settings for the ISR service."""
    
    # API settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "ISR Service"
    
    # CORS settings
    CORS_ORIGINS: List[str] = ["http://localhost:3000", "http://localhost:8080"]
    
    # Database settings
    POSTGRES_HOST: str = os.getenv("POSTGRES_HOST", "localhost")
    POSTGRES_PORT: str = os.getenv("POSTGRES_PORT", "5432")
    POSTGRES_USER: str = os.getenv("POSTGRES_USER", "postgres")
    POSTGRES_PASSWORD: str = os.getenv("POSTGRES_PASSWORD", "postgres")
    POSTGRES_DB: str = os.getenv("POSTGRES_DB", "isr_service")
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
    RABBITMQ_EXCHANGE: str = os.getenv("RABBITMQ_EXCHANGE", "bulo.sentinel.isr")
    
    # Redis settings
    REDIS_HOST: str = os.getenv("REDIS_HOST", "localhost")
    REDIS_PORT: int = int(os.getenv("REDIS_PORT", "6379"))
    REDIS_PASSWORD: str = os.getenv("REDIS_PASSWORD", "")
    REDIS_DB: int = int(os.getenv("REDIS_DB", "0"))
    
    # Sensor settings
    SENSOR_POLLING_INTERVAL: float = float(os.getenv("SENSOR_POLLING_INTERVAL", "1.0"))
    
    # Target tracking settings
    TARGET_TRACKING_CONFIDENCE_THRESHOLD: float = float(os.getenv("TARGET_TRACKING_CONFIDENCE_THRESHOLD", "0.5"))
    TARGET_TRACKING_MAX_AGE: int = int(os.getenv("TARGET_TRACKING_MAX_AGE", "30"))
    
    # Data fusion settings
    DATA_FUSION_CONFIDENCE_THRESHOLD: float = float(os.getenv("DATA_FUSION_CONFIDENCE_THRESHOLD", "0.7"))
    
    # Security settings
    RATE_LIMIT_REQUESTS: int = int(os.getenv("RATE_LIMIT_REQUESTS", "100"))
    RATE_LIMIT_WINDOW_SECONDS: int = int(os.getenv("RATE_LIMIT_WINDOW_SECONDS", "60"))
    
    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
