"""
Configuration utilities for the Counter-UAS module.

This module provides utilities for loading and accessing configuration.
"""

import os
import logging
import yaml
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)

# Global configuration
_config = {}


def load_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load configuration from a YAML file.
    
    Args:
        config_path: Path to the configuration file. If None, uses the default path.
        
    Returns:
        Dict[str, Any]: Configuration dictionary.
    """
    global _config
    
    # Default config path
    if config_path is None:
        config_path = os.environ.get("COUNTER_UAS_CONFIG", "config/counter_uas.yaml")
    
    logger.info(f"Loading configuration from {config_path}")
    
    try:
        # Check if the file exists
        if not os.path.exists(config_path):
            logger.warning(f"Configuration file {config_path} not found, using default configuration")
            _config = _get_default_config()
            return _config
        
        # Load the configuration file
        with open(config_path, "r") as f:
            _config = yaml.safe_load(f)
        
        logger.info(f"Configuration loaded successfully")
        return _config
    except Exception as e:
        logger.error(f"Failed to load configuration: {str(e)}")
        _config = _get_default_config()
        return _config


def get_config() -> Dict[str, Any]:
    """
    Get the current configuration.
    
    Returns:
        Dict[str, Any]: Configuration dictionary.
    """
    global _config
    
    # If the configuration is empty, load the default configuration
    if not _config:
        _config = load_config()
    
    return _config


def _get_default_config() -> Dict[str, Any]:
    """
    Get the default configuration.
    
    Returns:
        Dict[str, Any]: Default configuration dictionary.
    """
    return {
        "hardware": {
            "kerberos_sdr_devices": [
                {
                    "device_id": "0",
                    "center_frequency": 915000000,  # 915 MHz
                    "sample_rate": 2400000,  # 2.4 MSPS
                    "gain": 30.0,  # dB
                    "reference_channel": 0
                }
            ],
            "acconeer_radar_devices": [
                {
                    "device_id": "0",
                    "mode": "iq",
                    "start_range": 0.2,  # 0.2 meters
                    "end_range": 5.0,  # 5.0 meters
                    "update_rate": 10.0  # 10 Hz
                }
            ]
        },
        "processing": {
            "doa": {
                "algorithm": "MUSIC",
                "update_rate": 10.0,  # 10 Hz
                "smoothing_factor": 0.5
            },
            "radar": {
                "algorithm": "range_doppler",
                "update_rate": 10.0,  # 10 Hz
                "min_distance": 0.5,  # 0.5 meters
                "max_distance": 5.0,  # 5.0 meters
                "min_velocity": 0.1,  # 0.1 m/s
                "max_velocity": 10.0  # 10.0 m/s
            },
            "fusion": {
                "algorithm": "ekf",
                "update_rate": 10.0,  # 10 Hz
                "process_noise": 0.1,
                "measurement_noise": 0.1
            }
        },
        "detection": {
            "min_confidence": 0.5,
            "max_age": 5.0,  # 5.0 seconds
            "min_detections": 3
        },
        "events": {
            "rabbitmq": {
                "host": "localhost",
                "port": 5672,
                "username": "guest",
                "password": "guest",
                "exchange": "counter_uas",
                "exchange_type": "topic"
            }
        },
        "api": {
            "host": "0.0.0.0",
            "port": 8000,
            "debug": False,
            "cors_origins": ["*"]
        },
        "logging": {
            "level": "INFO",
            "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        }
    }
