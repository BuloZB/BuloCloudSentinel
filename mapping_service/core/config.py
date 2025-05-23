"""
Configuration settings for the Mapping Service.
"""

import os
from typing import Any, Dict, List, Optional, Union
from pydantic import BaseSettings, PostgresDsn, validator


class Settings(BaseSettings):
    """Application settings."""
    
    # API settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Bulo.Cloud Sentinel Mapping Service"
    
    # CORS settings
    BACKEND_CORS_ORIGINS: List[str] = ["*"]
    
    # Security settings
    SECRET_KEY: str = os.getenv("SECRET_KEY", "mapping_service_secret_key")
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 60 * 24 * 8  # 8 days
    
    # Database settings
    POSTGRES_SERVER: str = os.getenv("POSTGRES_SERVER", "localhost")
    POSTGRES_USER: str = os.getenv("POSTGRES_USER", "postgres")
    POSTGRES_PASSWORD: str = os.getenv("POSTGRES_PASSWORD", "postgres")
    POSTGRES_DB: str = os.getenv("POSTGRES_DB", "mapping_service")
    POSTGRES_PORT: str = os.getenv("POSTGRES_PORT", "5432")
    SQLALCHEMY_DATABASE_URI: Optional[PostgresDsn] = None
    
    @validator("SQLALCHEMY_DATABASE_URI", pre=True)
    def assemble_db_connection(cls, v: Optional[str], values: Dict[str, Any]) -> Any:
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
    
    # MinIO settings
    MINIO_SERVER: str = os.getenv("MINIO_SERVER", "localhost")
    MINIO_PORT: str = os.getenv("MINIO_PORT", "9000")
    MINIO_ACCESS_KEY: str = os.getenv("MINIO_ACCESS_KEY", "minioadmin")
    MINIO_SECRET_KEY: str = os.getenv("MINIO_SECRET_KEY", "minioadmin")
    MINIO_BUCKET: str = os.getenv("MINIO_BUCKET", "mapping-service")
    MINIO_URL: str = f"{MINIO_SERVER}:{MINIO_PORT}"
    
    # OpenDroneMap settings
    ODM_API_URL: str = os.getenv("ODM_API_URL", "http://localhost:3000")
    
    # Processing settings
    WORKER_CONCURRENCY: int = int(os.getenv("WORKER_CONCURRENCY", "2"))
    MAX_IMAGE_SIZE_MB: int = int(os.getenv("MAX_IMAGE_SIZE_MB", "50"))
    TEMP_DIRECTORY: str = os.getenv("TEMP_DIRECTORY", "/tmp/mapping_service")
    
    # Storage paths
    IMAGES_PATH: str = "images"
    ORTHOMOSAICS_PATH: str = "orthomosaics"
    MESHES_PATH: str = "meshes"
    POINT_CLOUDS_PATH: str = "point_clouds"
    TILES_PATH: str = "tiles"
    
    # Sentinel integration
    SENTINEL_API_URL: str = os.getenv("SENTINEL_API_URL", "http://localhost:8000")
    SENTINEL_API_TOKEN: str = os.getenv("SENTINEL_API_TOKEN", "")
    
    # Cesium settings
    CESIUM_ION_TOKEN: str = os.getenv("CESIUM_ION_TOKEN", "")
    
    class Config:
        case_sensitive = True


settings = Settings()
