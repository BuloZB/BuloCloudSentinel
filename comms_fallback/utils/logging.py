"""
Logging utilities.

This module provides utilities for setting up logging.
"""

import logging
import os
import sys
from typing import Optional


def setup_logging(
    log_level: str = "INFO",
    log_file: Optional[str] = None,
    log_format: Optional[str] = None,
):
    """
    Set up logging.

    Args:
        log_level: Log level
        log_file: Log file path
        log_format: Log format
    """
    # Set up log level
    numeric_level = getattr(logging, log_level.upper(), None)
    if not isinstance(numeric_level, int):
        numeric_level = logging.INFO

    # Set up log format
    if not log_format:
        log_format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"

    # Set up logging
    handlers = []

    # Add console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    handlers.append(console_handler)

    # Add file handler if log file is specified
    if log_file:
        # Create directory if it doesn't exist
        log_dir = os.path.dirname(log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)

        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(logging.Formatter(log_format))
        handlers.append(file_handler)

    # Configure logging
    logging.basicConfig(
        level=numeric_level,
        format=log_format,
        handlers=handlers,
    )

    # Set up module logger
    logger = logging.getLogger("comms_fallback")
    logger.setLevel(numeric_level)

    # Log configuration
    logger.info(f"Logging initialized with level {log_level}")
    if log_file:
        logger.info(f"Logging to file: {log_file}")

    return logger
