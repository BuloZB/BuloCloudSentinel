"""
Common logging configuration for the BuloCloud Sentinel project.

This module provides a consistent logging configuration across all modules.
"""

from __future__ import annotations

import json
import logging
import os
import sys
from typing import Any, Optional, Union

from loguru import logger


class InterceptHandler(logging.Handler):
    """
    Intercept standard logging messages and redirect them to loguru.

    This handler intercepts all standard logging messages and redirects them
    to loguru for consistent formatting and handling.
    """

    def emit(self, record: logging.LogRecord) -> None:
        """
        Intercept logging messages and redirect to loguru.

        Args:
            record: The log record to intercept
        """
        # Get corresponding Loguru level if it exists
        try:
            level = logger.level(record.levelname).name
        except ValueError:
            level = record.levelno

        # Find caller from where the logged message originated
        frame, depth = logging.currentframe(), 2
        while frame and frame.f_code.co_filename == logging.__file__:
            frame = frame.f_back
            depth += 1

        logger.opt(depth=depth, exception=record.exc_info).log(
            level, record.getMessage()
        )


class JSONSink:
    """
    JSON sink for loguru.

    This sink formats log messages as JSON for structured logging.
    """

    def __init__(self, file_path: str) -> None:
        """
        Initialize the JSON sink.

        Args:
            file_path: Path to the log file
        """
        self.file_path = file_path
        os.makedirs(os.path.dirname(file_path), exist_ok=True)

    def __call__(self, message: Any) -> None:
        """
        Format and write log message as JSON.

        Args:
            message: The log message to format and write
        """
        record = message.record
        log_entry = {
            "timestamp": record["time"].isoformat(),
            "level": record["level"].name,
            "message": record["message"],
            "module": record["name"],
            "function": record["function"],
            "line": record["line"],
            "process_id": record["process"].id,
            "thread_id": record["thread"].id,
        }

        # Add exception info if available
        if record["exception"]:
            log_entry["exception"] = str(record["exception"])

        # Add extra attributes
        for key, value in record["extra"].items():
            log_entry[key] = value

        # Write to file
        with open(self.file_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(log_entry) + "\n")


def setup_logging(
    log_level: Union[str, int] = "INFO",
    json_logs: bool = False,
    log_file: Optional[str] = None,
) -> None:
    """
    Set up logging configuration.

    Args:
        log_level: The log level (default: "INFO")
        json_logs: Whether to use JSON logging (default: False)
        log_file: Path to the log file (default: None)
    """
    # Remove default loguru handler
    logger.remove()

    # Add console handler
    logger.add(
        sys.stderr,
        level=log_level,
        format=(
            "<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | "
            "<level>{level: <8}</level> | "
            "<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> | "
            "<level>{message}</level>"
        ),
        colorize=True,
    )

    # Add file handler if specified
    if log_file:
        if json_logs:
            # Add JSON sink
            logger.add(JSONSink(log_file), level=log_level)
        else:
            # Add regular file sink
            logger.add(
                log_file,
                level=log_level,
                format=(
                    "{time:YYYY-MM-DD HH:mm:ss.SSS} | "
                    "{level: <8} | "
                    "{name}:{function}:{line} | "
                    "{message}"
                ),
                rotation="10 MB",
                retention="1 month",
            )

    # Intercept standard logging
    logging.basicConfig(handlers=[InterceptHandler()], level=0, force=True)

    # Replace standard library logging with loguru
    for name in logging.root.manager.loggerDict:
        logging.getLogger(name).handlers = [InterceptHandler()]

    # Set log level for specific modules
    logging.getLogger("sqlalchemy.engine").setLevel(logging.WARNING)
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger for the specified name.

    Args:
        name: The logger name

    Returns:
        The logger
    """
    return logging.getLogger(name)
