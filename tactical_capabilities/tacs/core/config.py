"""
Configuration settings for the TACS module.
"""

import os
from typing import List, Optional, Union, Any
from pydantic import BaseSettings, PostgresDsn, field_validator

class Settings(BaseSettings):
    """Application settings."""

    # API settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Target Acquisition and Coordination System"

    # CORS settings
    CORS_ORIGINS: List[str] = []

    @field_validator("CORS_ORIGINS", mode="before")
    def assemble_cors_origins(cls, v: Union[str, List[str]]) -> List[str]:
        """Parse CORS origins from environment variable."""
        if isinstance(v, str) and not v.startswith("["):
            return [origin.strip() for origin in v.split(",")]
        elif isinstance(v, (list, str)):
            return v
        raise ValueError(v)

    # Security settings
    SECRET_KEY: str = os.getenv("SECRET_KEY")
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    @field_validator("SECRET_KEY")
    def validate_secret_key(cls, v):
        if not v:
            raise ValueError("SECRET_KEY environment variable is required")
        if len(v) < 32:
            raise ValueError("SECRET_KEY must be at least 32 characters long")
        return v

    # Rate limiting
    RATE_LIMIT_REQUESTS: int = 100
    RATE_LIMIT_WINDOW_SECONDS: int = 60

    # Database settings
    POSTGRES_SERVER: str = os.getenv("POSTGRES_HOST", "localhost")
    POSTGRES_PORT: str = os.getenv("POSTGRES_PORT", "5432")
    POSTGRES_USER: str = os.getenv("POSTGRES_USER", "postgres")
    POSTGRES_PASSWORD: str = os.getenv("POSTGRES_PASSWORD", "postgres")
    POSTGRES_DB: str = os.getenv("POSTGRES_DB", "tacs")
    SQLALCHEMY_DATABASE_URI: Optional[PostgresDsn] = None

    @field_validator("SQLALCHEMY_DATABASE_URI", mode="before")
    def assemble_db_connection(cls, v: Optional[str], info) -> Any:
        """Assemble database connection string."""
        if isinstance(v, str):
            return v
        values = info.data
        return PostgresDsn.build(
            scheme="postgresql+asyncpg",
            user=values.get("POSTGRES_USER"),
            password=values.get("POSTGRES_PASSWORD"),
            host=values.get("POSTGRES_SERVER"),
            port=values.get("POSTGRES_PORT"),
            path=f"/{values.get('POSTGRES_DB') or ''}",
        )

    # RabbitMQ settings
    RABBITMQ_HOST: str = os.getenv("RABBITMQ_HOST", "localhost")
    RABBITMQ_PORT: int = int(os.getenv("RABBITMQ_PORT", "5672"))
    RABBITMQ_USER: str = os.getenv("RABBITMQ_USER", "guest")
    RABBITMQ_PASSWORD: str = os.getenv("RABBITMQ_PASSWORD", "guest")
    RABBITMQ_EXCHANGE: str = os.getenv("RABBITMQ_EXCHANGE", "bulo.sentinel.tacs")

    # Redis settings
    REDIS_HOST: str = os.getenv("REDIS_HOST", "localhost")
    REDIS_PORT: int = int(os.getenv("REDIS_PORT", "6379"))
    REDIS_PASSWORD: Optional[str] = os.getenv("REDIS_PASSWORD")
    REDIS_DB: int = int(os.getenv("REDIS_DB", "0"))

    # Storage settings
    STORAGE_TYPE: str = os.getenv("STORAGE_TYPE", "local")  # local, s3
    LOCAL_STORAGE_PATH: str = os.getenv("LOCAL_STORAGE_PATH", "/tmp/tacs")
    S3_BUCKET: str = os.getenv("S3_BUCKET", "tacs")
    S3_REGION: str = os.getenv("S3_REGION", "us-east-1")
    S3_ENDPOINT: Optional[str] = os.getenv("S3_ENDPOINT")
    S3_ACCESS_KEY: Optional[str] = os.getenv("S3_ACCESS_KEY")
    S3_SECRET_KEY: Optional[str] = os.getenv("S3_SECRET_KEY")

    @field_validator("S3_ACCESS_KEY", "S3_SECRET_KEY")
    def validate_s3_credentials(cls, v, info):
        field_name = info.field_name
        if info.data.get("STORAGE_TYPE") == "s3" and not v:
            raise ValueError(f"{field_name} is required when STORAGE_TYPE is 's3'")
        return v

    # Keycloak settings
    KEYCLOAK_URL: str = os.getenv("KEYCLOAK_URL", "http://localhost:8080")
    KEYCLOAK_REALM: str = os.getenv("KEYCLOAK_REALM", "bulo-sentinel")
    KEYCLOAK_CLIENT_ID: str = os.getenv("KEYCLOAK_CLIENT_ID", "tacs")
    KEYCLOAK_CLIENT_SECRET: Optional[str] = os.getenv("KEYCLOAK_CLIENT_SECRET")

    @field_validator("KEYCLOAK_CLIENT_SECRET")
    def validate_keycloak_client_secret(cls, v, info):
        if info.data.get("KEYCLOAK_URL") and info.data.get("KEYCLOAK_REALM") and info.data.get("KEYCLOAK_CLIENT_ID") and not v:
            raise ValueError("KEYCLOAK_CLIENT_SECRET is required when Keycloak is configured")

    # Logging settings
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")

    # Target tracking settings
    TARGET_ASSOCIATION_THRESHOLD: float = float(os.getenv("TARGET_ASSOCIATION_THRESHOLD", "100.0"))  # meters
    TARGET_TRACK_TIMEOUT: int = int(os.getenv("TARGET_TRACK_TIMEOUT", "300"))  # seconds

    # Sensor fusion settings
    FUSION_DEFAULT_METHOD: str = os.getenv("FUSION_DEFAULT_METHOD", "weighted_average")

    # Coordination settings
    COORDINATION_MAX_PLATFORMS: int = int(os.getenv("COORDINATION_MAX_PLATFORMS", "10"))

    class Config:
        """Pydantic config."""
        case_sensitive = True
        env_file = ".env"

# Create settings instance
settings = Settings()
