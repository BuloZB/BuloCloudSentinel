"""
Configuration utilities for the AI Analytics module.
"""

import os
import yaml
import json
from typing import Dict, Any

class Config:
    """Configuration manager."""
    
    @staticmethod
    def load_yaml(file_path: str) -> Dict[str, Any]:
        """Load configuration from a YAML file."""
        with open(file_path, "r") as f:
            return yaml.safe_load(f)
    
    @staticmethod
    def load_json(file_path: str) -> Dict[str, Any]:
        """Load configuration from a JSON file."""
        with open(file_path, "r") as f:
            return json.load(f)
    
    @staticmethod
    def get_env(name: str, default: Any = None) -> Any:
        """Get an environment variable."""
        return os.environ.get(name, default)
    
    @staticmethod
    def get_env_bool(name: str, default: bool = False) -> bool:
        """Get a boolean environment variable."""
        value = os.environ.get(name)
        if value is None:
            return default
        return value.lower() in ("true", "1", "yes", "y", "t")
    
    @staticmethod
    def get_env_int(name: str, default: int = 0) -> int:
        """Get an integer environment variable."""
        value = os.environ.get(name)
        if value is None:
            return default
        try:
            return int(value)
        except ValueError:
            return default
    
    @staticmethod
    def get_env_float(name: str, default: float = 0.0) -> float:
        """Get a float environment variable."""
        value = os.environ.get(name)
        if value is None:
            return default
        try:
            return float(value)
        except ValueError:
            return default

def load_config() -> Dict[str, Any]:
    """Load the application configuration."""
    # Get configuration file path from environment variable
    config_path = os.environ.get("CONFIG_PATH", "config/config.yaml")
    
    # Check if file exists
    if os.path.exists(config_path):
        # Load configuration from file
        if config_path.endswith(".yaml") or config_path.endswith(".yml"):
            config = Config.load_yaml(config_path)
        elif config_path.endswith(".json"):
            config = Config.load_json(config_path)
        else:
            raise ValueError(f"Unsupported configuration file format: {config_path}")
    else:
        # Use default configuration
        config = {
            "video_streams": {
                "frame_buffer_size": 10,
                "reconnect_interval": 5
            },
            "event_broker": {
                "type": "redis",
                "host": "redis",
                "port": 6379,
                "channel_prefix": "ai_analytics"
            },
            "detection": {
                "default_model": "yolov8n",
                "confidence_threshold": 0.5,
                "max_detection_fps": 10
            },
            "recognition": {
                "face_recognition_threshold": 0.7,
                "license_plate_confidence_threshold": 0.8,
                "max_recognition_fps": 5
            },
            "behavior": {
                "loitering_threshold": 60,
                "crowd_threshold": 5,
                "running_speed_threshold": 2.0,
                "direction_change_threshold": 90.0,
                "max_analysis_fps": 5
            },
            "analytics": {
                "data_retention_days": 90,
                "training_interval_hours": 24
            }
        }
    
    # Override configuration with environment variables
    # Example: AI_ANALYTICS_DETECTION_CONFIDENCE_THRESHOLD=0.6
    for key in config.keys():
        env_prefix = f"AI_ANALYTICS_{key.upper()}_"
        for env_key, env_value in os.environ.items():
            if env_key.startswith(env_prefix):
                # Extract config key
                config_key = env_key[len(env_prefix):].lower()
                
                # Update config
                if config_key in config[key]:
                    # Convert value to appropriate type
                    if isinstance(config[key][config_key], bool):
                        config[key][config_key] = env_value.lower() in ("true", "1", "yes", "y", "t")
                    elif isinstance(config[key][config_key], int):
                        config[key][config_key] = int(env_value)
                    elif isinstance(config[key][config_key], float):
                        config[key][config_key] = float(env_value)
                    else:
                        config[key][config_key] = env_value
    
    return config
