"""
Common utility functions for the BuloCloud Sentinel project.

This module provides utility functions that are used across multiple modules.
"""

from __future__ import annotations

import functools
import logging
import time
from datetime import datetime, UTC
from typing import Any, Callable, TypeVar

# Type variables for generic functions
T = TypeVar("T")
R = TypeVar("R")

logger = logging.getLogger(__name__)


def timed(func: Callable[..., R]) -> Callable[..., R]:
    """
    Decorator to measure the execution time of a function.

    Args:
        func: The function to measure

    Returns:
        The wrapped function
    """
    @functools.wraps(func)
    def wrapper(*args: Any, **kwargs: Any) -> R:
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        logger.debug(
            "Function %s executed in %.2f seconds",
            func.__name__,
            end_time - start_time,
        )
        return result
    return wrapper


def async_timed(func: Callable[..., R]) -> Callable[..., R]:
    """
    Decorator to measure the execution time of an async function.

    Args:
        func: The async function to measure

    Returns:
        The wrapped async function
    """
    @functools.wraps(func)
    async def wrapper(*args: Any, **kwargs: Any) -> R:
        start_time = time.time()
        result = await func(*args, **kwargs)
        end_time = time.time()
        logger.debug(
            "Async function %s executed in %.2f seconds",
            func.__name__,
            end_time - start_time,
        )
        return result
    return wrapper


def utc_now() -> datetime:
    """
    Get the current UTC datetime.

    Returns:
        Current UTC datetime
    """
    return datetime.now(UTC)


def format_datetime(dt: datetime, fmt: str = "%Y-%m-%d %H:%M:%S") -> str:
    """
    Format a datetime object as a string.

    Args:
        dt: The datetime to format
        fmt: The format string (default: "%Y-%m-%d %H:%M:%S")

    Returns:
        Formatted datetime string
    """
    return dt.strftime(fmt)


def parse_datetime(dt_str: str, fmt: str = "%Y-%m-%d %H:%M:%S") -> datetime:
    """
    Parse a datetime string into a datetime object.

    Args:
        dt_str: The datetime string to parse
        fmt: The format string (default: "%Y-%m-%d %H:%M:%S")

    Returns:
        Parsed datetime object with UTC timezone
    """
    dt = datetime.strptime(dt_str, fmt)
    return dt.replace(tzinfo=UTC)


def truncate_string(s: str, max_length: int = 100, suffix: str = "...") -> str:
    """
    Truncate a string to a maximum length.

    Args:
        s: The string to truncate
        max_length: Maximum length of the string (default: 100)
        suffix: Suffix to add to truncated string (default: "...")

    Returns:
        Truncated string
    """
    if len(s) <= max_length:
        return s
    return s[:max_length - len(suffix)] + suffix


def safe_divide(numerator: float, denominator: float, default: float = 0.0) -> float:
    """
    Safely divide two numbers, returning a default value if the denominator is zero.

    Args:
        numerator: The numerator
        denominator: The denominator
        default: Default value to return if denominator is zero (default: 0.0)

    Returns:
        Result of division or default value
    """
    if denominator == 0:
        return default
    return numerator / denominator
