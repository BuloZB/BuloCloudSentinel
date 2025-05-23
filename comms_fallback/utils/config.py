"""
Configuration utilities.

This module provides utilities for loading and validating configuration.
"""

import json
import logging
import os
import re
from typing import Any, Dict, Optional

import yaml

logger = logging.getLogger(__name__)

# Global configuration
_config = {}


def load_config(config_path: str) -> Dict[str, Any]:
    """
    Load configuration from a YAML file.

    Args:
        config_path: Path to configuration file

    Returns:
        Configuration dictionary
    """
    global _config
    
    try:
        logger.info(f"Loading configuration from {config_path}")
        
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        
        # Replace environment variables
        config_str = json.dumps(config)
        env_var_pattern = re.compile(r"\$\{([^}^{]+)\}")
        
        def replace_env_var(match):
            env_var = match.group(1)
            if ":" in env_var:
                env_var, default = env_var.split(":", 1)
                return os.environ.get(env_var, default)
            else:
                return os.environ.get(env_var, "")
        
        config_str = env_var_pattern.sub(replace_env_var, config_str)
        config = json.loads(config_str)
        
        # Update global configuration
        _config.update(config)
        
        return config
        
    except Exception as e:
        logger.error(f"Error loading configuration: {e}")
        return {}


def get_config() -> Dict[str, Any]:
    """
    Get the current configuration.

    Returns:
        Configuration dictionary
    """
    global _config
    return _config


def validate_config(config: Dict[str, Any]) -> bool:
    """
    Validate configuration.

    Args:
        config: Configuration dictionary

    Returns:
        True if configuration is valid, False otherwise
    """
    try:
        # Check required sections
        required_sections = ["adapters", "monitor", "fallback", "wireguard"]
        for section in required_sections:
            if section not in config:
                logger.error(f"Missing required configuration section: {section}")
                return False
        
        # Check adapters
        adapters = config["adapters"]
        if not adapters:
            logger.error("No adapters configured")
            return False
        
        for adapter_id, adapter_config in adapters.items():
            if not adapter_config.get("enabled", True):
                continue
            
            # Check required adapter fields
            required_fields = ["priority"]
            for field in required_fields:
                if field not in adapter_config:
                    logger.error(f"Missing required field {field} in adapter {adapter_id}")
                    return False
        
        # Check monitor
        monitor = config["monitor"]
        required_fields = ["check_interval", "hysteresis_time", "hysteresis_count"]
        for field in required_fields:
            if field not in monitor:
                logger.error(f"Missing required field {field} in monitor configuration")
                return False
        
        # Check fallback
        fallback = config["fallback"]
        required_fields = ["buffer_size", "critical_timeout", "session_timeout"]
        for field in required_fields:
            if field not in fallback:
                logger.error(f"Missing required field {field} in fallback configuration")
                return False
        
        # Check wireguard
        wireguard = config["wireguard"]
        if wireguard.get("enabled", True):
            required_fields = ["interface", "port", "peers"]
            for field in required_fields:
                if field not in wireguard:
                    logger.error(f"Missing required field {field} in wireguard configuration")
                    return False
        
        return True
        
    except Exception as e:
        logger.error(f"Error validating configuration: {e}")
        return False
