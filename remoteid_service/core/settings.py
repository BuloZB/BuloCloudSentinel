"""
Settings module for the Remote ID & Regulatory Compliance Service.

This module provides configuration settings for the service, loaded from
environment variables and/or configuration files.
"""

import os
import yaml
from functools import lru_cache
from typing import Dict, List, Optional, Union, Any

from pydantic import (
    AnyHttpUrl,
    PostgresDsn,
    field_validator,
    model_validator,
)
from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    """
    Settings for the Remote ID & Regulatory Compliance Service.
    
    This class defines all configuration settings for the service, with defaults
    that can be overridden by environment variables or a configuration file.
    """
    # General settings
    PROJECT_NAME: str = "Remote ID & Regulatory Compliance Service"
    API_V1_STR: str = "/api/v1"
    ENVIRONMENT: str = "development"
    DEBUG: bool = False
    LOG_LEVEL: str = "INFO"
    
    # CORS settings
    BACKEND_CORS_ORIGINS: List[AnyHttpUrl] = []
    
    # Database settings
    DATABASE_URL: PostgresDsn
    DATABASE_POOL_SIZE: int = 5
    DATABASE_MAX_OVERFLOW: int = 10
    DATABASE_POOL_RECYCLE: int = 3600
    
    # Redis settings
    REDIS_URL: str = "redis://:placeholderpassword@localhost:6379/0"
    
    # Security settings
    SECRET_KEY: str = "placeholdersecretkey"
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30
    
    # Remote ID settings
    ENABLE_BROADCAST: bool = False
    ENABLE_HARDWARE: bool = False
    BROADCAST_INTERVAL: int = 1  # seconds
    BROADCAST_METHODS: List[str] = ["wifi_nan", "bluetooth_le"]
    DEFAULT_BROADCAST_MODE: str = "faa"  # faa or eu
    
    # Flight plan settings
    EASA_API_URL: Optional[str] = None
    EASA_API_KEY: Optional[str] = None
    FAA_LAANC_API_URL: Optional[str] = None
    FAA_LAANC_API_KEY: Optional[str] = None
    
    # NOTAM settings
    NOTAM_UPDATE_INTERVAL: int = 3600  # seconds
    NOTAM_SOURCES: List[str] = ["faa", "easa"]
    
    # Logging settings
    LOG_RETENTION_DAYS: int = 7
    BROADCAST_LOG_RETENTION_HOURS: int = 24
    
    # Hardware settings
    BLUETOOTH_ADAPTER: str = "hci0"
    WIFI_ADAPTER: str = "wlan0"
    
    # Model config
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        case_sensitive=True,
    )
    
    @field_validator("BACKEND_CORS_ORIGINS")
    def assemble_cors_origins(cls, v: Union[str, List[str]]) -> Union[List[str], str]:
        """
        Validate and process CORS origins.
        
        Args:
            v: CORS origins as string or list
            
        Returns:
            List of CORS origins
        """
        if isinstance(v, str) and not v.startswith("["):
            return [i.strip() for i in v.split(",")]
        elif isinstance(v, (list, str)):
            return v
        raise ValueError(v)
    
    @model_validator(mode="after")
    def validate_broadcast_settings(self) -> "Settings":
        """
        Validate broadcast settings.
        
        Returns:
            Settings object
        """
        if self.ENABLE_BROADCAST and not self.ENABLE_HARDWARE:
            # If broadcasting is enabled but hardware is not, log a warning
            print("WARNING: Broadcasting is enabled but hardware support is disabled. "
                  "Broadcasting will be simulated.")
        
        return self

def load_config_from_file(file_path: str) -> Dict[str, Any]:
    """
    Load configuration from a YAML file.
    
    Args:
        file_path: Path to the configuration file
        
    Returns:
        Dict containing configuration values
    """
    if not os.path.exists(file_path):
        return {}
    
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

@lru_cache()
def get_settings() -> Settings:
    """
    Get settings singleton.
    
    This function returns a cached instance of the Settings class,
    loading values from environment variables and/or a configuration file.
    
    Returns:
        Settings object
    """
    # Load configuration from file if it exists
    config_file = os.environ.get("CONFIG_FILE", "config/config.yaml")
    config_values = load_config_from_file(config_file)
    
    # Create settings object, with environment variables taking precedence
    return Settings(**config_values)
