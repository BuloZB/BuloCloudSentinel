"""
Configuration settings for the SIGINT service.
"""

import os
from typing import List, Optional
from pydantic import PostgresDsn, field_validator
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    """Settings for the SIGINT service."""

    # API settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "SIGINT Service"

    # CORS settings
    CORS_ORIGINS: List[str] = ["http://localhost:3000", "http://localhost:8080"]

    # Database settings
    POSTGRES_HOST: str = os.getenv("POSTGRES_HOST", "localhost")
    POSTGRES_PORT: str = os.getenv("POSTGRES_PORT", "5432")
    POSTGRES_USER: str = os.getenv("POSTGRES_USER", "postgres")
    POSTGRES_PASSWORD: str = os.getenv("POSTGRES_PASSWORD", "postgres")
    POSTGRES_DB: str = os.getenv("POSTGRES_DB", "sigint_service")
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
    RABBITMQ_EXCHANGE: str = os.getenv("RABBITMQ_EXCHANGE", "bulo.sentinel.sigint")

    # Redis settings
    REDIS_HOST: str = os.getenv("REDIS_HOST", "localhost")
    REDIS_PORT: int = int(os.getenv("REDIS_PORT", "6379"))
    REDIS_PASSWORD: str = os.getenv("REDIS_PASSWORD", "")
    REDIS_DB: int = int(os.getenv("REDIS_DB", "0"))

    # MinIO settings
    MINIO_ENDPOINT: str = os.getenv("MINIO_ENDPOINT", "localhost:9000")
    MINIO_ACCESS_KEY: str = os.getenv("MINIO_ACCESS_KEY", "minioadmin")
    MINIO_SECRET_KEY: str = os.getenv("MINIO_SECRET_KEY", "minioadmin")
    MINIO_BUCKET_NAME: str = os.getenv("MINIO_BUCKET_NAME", "sigint")
    MINIO_SECURE: bool = os.getenv("MINIO_SECURE", "False").lower() == "true"

    # Collector settings
    COLLECTOR_POLLING_INTERVAL: float = float(os.getenv("COLLECTOR_POLLING_INTERVAL", "1.0"))

    # Signal analysis settings
    SIGNAL_ANALYSIS_CONFIDENCE_THRESHOLD: float = float(os.getenv("SIGNAL_ANALYSIS_CONFIDENCE_THRESHOLD", "0.5"))
    SIGNAL_ANALYSIS_MAX_DURATION: int = int(os.getenv("SIGNAL_ANALYSIS_MAX_DURATION", "60"))  # seconds

    # Direction finding settings
    DIRECTION_FINDING_MIN_COLLECTORS: int = int(os.getenv("DIRECTION_FINDING_MIN_COLLECTORS", "2"))
    DIRECTION_FINDING_MAX_AGE: int = int(os.getenv("DIRECTION_FINDING_MAX_AGE", "60"))  # seconds

    # Threat detection settings
    THREAT_DETECTION_CONFIDENCE_THRESHOLD: float = float(os.getenv("THREAT_DETECTION_CONFIDENCE_THRESHOLD", "0.7"))

    # Security settings
    RATE_LIMIT_REQUESTS: int = int(os.getenv("RATE_LIMIT_REQUESTS", "100"))
    RATE_LIMIT_WINDOW_SECONDS: int = int(os.getenv("RATE_LIMIT_WINDOW_SECONDS", "60"))

    # Signal recording settings
    MAX_RECORDING_DURATION: int = int(os.getenv("MAX_RECORDING_DURATION", "300"))  # seconds
    MAX_RECORDING_SIZE: int = int(os.getenv("MAX_RECORDING_SIZE", "100"))  # MB

    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
