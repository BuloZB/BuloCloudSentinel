"""
Configuration settings for the TACS module.
"""

import os
from typing import List, Optional, Dict, Any
from pydantic import BaseSettings, PostgresDsn, validator

class Settings(BaseSettings):
    """Application settings."""
    
    # API settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Target Acquisition and Coordination System"
    
    # CORS settings
    CORS_ORIGINS: List[str] = ["*"]
    
    # Security settings
    SECRET_KEY: str = os.getenv("SECRET_KEY", "supersecretkey")
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30
    
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
    
    @validator("SQLALCHEMY_DATABASE_URI", pre=True)
    def assemble_db_connection(cls, v: Optional[str], values: Dict[str, Any]) -> Any:
        """Assemble database connection string."""
        if isinstance(v, str):
            return v
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
    
    # Keycloak settings
    KEYCLOAK_URL: str = os.getenv("KEYCLOAK_URL", "http://localhost:8080")
    KEYCLOAK_REALM: str = os.getenv("KEYCLOAK_REALM", "bulo-sentinel")
    KEYCLOAK_CLIENT_ID: str = os.getenv("KEYCLOAK_CLIENT_ID", "tacs")
    KEYCLOAK_CLIENT_SECRET: Optional[str] = os.getenv("KEYCLOAK_CLIENT_SECRET")
    
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
