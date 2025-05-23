"""
SentinelWeb Backend - Configuration

This module defines the configuration settings for the SentinelWeb backend service.
It uses Pydantic's BaseSettings for environment variable validation and loading.
"""

from typing import List, Optional, Union
from pydantic import BaseSettings, AnyHttpUrl, validator, PostgresDsn, RedisDsn

class Settings(BaseSettings):
    # API settings
    API_V1_STR: str = "/api/v1"

    # Security settings
    JWT_SECRET: str
    JWT_ALGORITHM: str = "HS256"
    JWT_EXPIRATION_MINUTES: int = 30  # 30 minutes
    JWT_REFRESH_TOKEN_EXPIRES_DAYS: int = 7  # 7 days

    @validator("JWT_SECRET")
    def validate_jwt_secret(cls, v):
        if len(v) < 32:
            raise ValueError("JWT_SECRET must be at least 32 characters long")
        return v

    # Host and CORS settings
    ALLOWED_HOSTS: List[str] = ["localhost", "127.0.0.1", "app.bulo.cloud"]
    CORS_ORIGINS: List[str] = ["http://localhost:3000", "https://app.bulo.cloud"]
    CORS_ALLOW_CREDENTIALS: bool = True

    @validator("ALLOWED_HOSTS", pre=True)
    def assemble_allowed_hosts(cls, v: Union[str, List[str]]) -> List[str]:
        if isinstance(v, str) and not v.startswith("["):
            return [i.strip() for i in v.split(",")]
        elif isinstance(v, (list, str)):
            return v
        raise ValueError(v)

    @validator("CORS_ORIGINS", pre=True)
    def assemble_cors_origins(cls, v: Union[str, List[str]]) -> List[str]:
        if isinstance(v, str) and not v.startswith("["):
            return [i.strip() for i in v.split(",")]
        elif isinstance(v, (list, str)):
            return v
        raise ValueError(v)

    # Database settings
    DATABASE_URL: PostgresDsn

    # Redis settings
    REDIS_URL: Optional[RedisDsn] = None

    # BuloCloudSentinel integration
    SENTINEL_API_URL: AnyHttpUrl
    SENTINEL_API_TOKEN: Optional[str] = None

    # RTMP server settings
    RTMP_SERVER_URL: Optional[str] = None

    # File storage
    UPLOAD_DIR: str = "./uploads"
    MAX_UPLOAD_SIZE: int = 100 * 1024 * 1024  # 100 MB

    # Logging
    LOG_LEVEL: str = "INFO"

    # WebSocket settings
    WS_PING_INTERVAL: int = 30  # seconds

    # Plugin settings
    PLUGINS_DIR: str = "./plugins"
    ENABLE_PLUGINS: bool = True

    # PWA settings
    PWA_ENABLED: bool = True

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = True


# Create settings instance
settings = Settings()
