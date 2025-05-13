"""
Common utilities for the BuloCloud Sentinel project.

This package provides common utilities, exceptions, validators, and logging
configuration that can be used across all modules.
"""

from __future__ import annotations

from common.exceptions import (
    AuthenticationError,
    AuthorizationError,
    ConfigurationError,
    RateLimitError,
    ResourceNotFoundError,
    SentinelError,
    ServiceUnavailableError,
    ValidationError,
)
from common.logging import get_logger, setup_logging
from common.utils import (
    async_timed,
    format_datetime,
    parse_datetime,
    safe_divide,
    timed,
    truncate_string,
    utc_now,
)
from common.validators import (
    GeoCoordinates,
    validate_enum,
    validate_future_date,
    validate_latitude,
    validate_longitude,
    validate_numeric_range,
    validate_regex,
    validate_string_length,
)

__all__ = [
    # Exceptions
    "SentinelError",
    "ConfigurationError",
    "ValidationError",
    "AuthenticationError",
    "AuthorizationError",
    "ResourceNotFoundError",
    "ServiceUnavailableError",
    "RateLimitError",
    
    # Logging
    "setup_logging",
    "get_logger",
    
    # Utils
    "timed",
    "async_timed",
    "utc_now",
    "format_datetime",
    "parse_datetime",
    "truncate_string",
    "safe_divide",
    
    # Validators
    "validate_latitude",
    "validate_longitude",
    "validate_future_date",
    "validate_string_length",
    "validate_regex",
    "validate_enum",
    "validate_numeric_range",
    "GeoCoordinates",
]
