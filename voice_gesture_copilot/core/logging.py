"""
Logging configuration for Voice & Gesture Co-Pilot.

This module provides logging configuration for the Voice & Gesture Co-Pilot module.
"""

import os
import sys
import logging
from pathlib import Path
from datetime import datetime

from loguru import logger

from voice_gesture_copilot.core.config import settings

# Create logs directory if it doesn't exist
logs_dir = Path("logs")
logs_dir.mkdir(exist_ok=True)

# Generate log filename with timestamp
log_filename = f"voice_gesture_copilot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
log_filepath = logs_dir / log_filename

class InterceptHandler(logging.Handler):
    """
    Intercept standard logging messages toward Loguru.
    
    This handler intercepts all standard logging messages and redirects them to Loguru.
    """
    
    def emit(self, record):
        # Get corresponding Loguru level if it exists
        try:
            level = logger.level(record.levelname).name
        except ValueError:
            level = record.levelno
        
        # Find caller from where the logged message originated
        frame, depth = logging.currentframe(), 2
        while frame.f_code.co_filename == logging.__file__:
            frame = frame.f_back
            depth += 1
        
        logger.opt(depth=depth, exception=record.exc_info).log(level, record.getMessage())

def setup_logging():
    """
    Configure logging for the application.
    
    This function sets up logging with Loguru, configuring console and file outputs
    with appropriate log levels and formats.
    """
    # Remove default loguru handler
    logger.remove()
    
    # Configure loguru
    log_format = (
        "<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | "
        "<level>{level: <8}</level> | "
        "<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - "
        "<level>{message}</level>"
    )
    
    # Add console handler
    logger.add(
        sys.stderr,
        format=log_format,
        level=settings.LOG_LEVEL,
        colorize=True,
    )
    
    # Add file handler
    logger.add(
        log_filepath,
        format=log_format,
        level=settings.LOG_LEVEL,
        rotation="10 MB",
        retention="1 week",
        compression="zip",
    )
    
    # Intercept standard logging
    logging.basicConfig(handlers=[InterceptHandler()], level=0, force=True)
    
    # Update logging levels for some noisy loggers
    for logger_name in [
        "uvicorn",
        "uvicorn.error",
        "uvicorn.access",
        "fastapi",
        "httpx",
    ]:
        logging.getLogger(logger_name).setLevel(logging.INFO)
    
    # Log startup message
    logger.info(f"Logging initialized (level: {settings.LOG_LEVEL})")
    logger.info(f"Log file: {log_filepath}")
    
    return logger
