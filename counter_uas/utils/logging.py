"""
Logging utilities for the Counter-UAS module.

This module provides utilities for setting up logging.
"""

import logging
import sys
from typing import Dict, Any, Optional

from counter_uas.utils.config import get_config


def setup_logging(config: Optional[Dict[str, Any]] = None) -> None:
    """
    Set up logging based on configuration.
    
    Args:
        config: Optional configuration dictionary. If None, uses the global configuration.
    """
    if config is None:
        config = get_config().get("logging", {})
    
    # Get logging configuration
    level_name = config.get("level", "INFO")
    log_format = config.get("format", "%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    
    # Convert level name to logging level
    level = getattr(logging, level_name.upper(), logging.INFO)
    
    # Configure root logger
    logging.basicConfig(
        level=level,
        format=log_format,
        stream=sys.stdout
    )
    
    # Configure specific loggers
    for logger_name, logger_config in config.get("loggers", {}).items():
        logger = logging.getLogger(logger_name)
        logger_level_name = logger_config.get("level", level_name)
        logger_level = getattr(logging, logger_level_name.upper(), level)
        logger.setLevel(logger_level)
    
    # Log configuration
    logging.info(f"Logging configured with level {level_name}")


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name.
    
    Args:
        name: Logger name.
        
    Returns:
        logging.Logger: Logger instance.
    """
    return logging.getLogger(name)
