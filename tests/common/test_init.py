"""
Tests for the common package.

This module contains tests for the common package.
"""

from __future__ import annotations

import pytest

import common


def test_imports() -> None:
    """Test that all expected symbols are imported."""
    # Check exceptions
    assert hasattr(common, "SentinelError")
    assert hasattr(common, "ConfigurationError")
    assert hasattr(common, "ValidationError")
    assert hasattr(common, "AuthenticationError")
    assert hasattr(common, "AuthorizationError")
    assert hasattr(common, "ResourceNotFoundError")
    assert hasattr(common, "ServiceUnavailableError")
    assert hasattr(common, "RateLimitError")
    
    # Check logging
    assert hasattr(common, "setup_logging")
    assert hasattr(common, "get_logger")
    
    # Check utils
    assert hasattr(common, "timed")
    assert hasattr(common, "async_timed")
    assert hasattr(common, "utc_now")
    assert hasattr(common, "format_datetime")
    assert hasattr(common, "parse_datetime")
    assert hasattr(common, "truncate_string")
    assert hasattr(common, "safe_divide")
    
    # Check validators
    assert hasattr(common, "validate_latitude")
    assert hasattr(common, "validate_longitude")
    assert hasattr(common, "validate_future_date")
    assert hasattr(common, "validate_string_length")
    assert hasattr(common, "validate_regex")
    assert hasattr(common, "validate_enum")
    assert hasattr(common, "validate_numeric_range")
    assert hasattr(common, "GeoCoordinates")


def test_all_variable() -> None:
    """Test that __all__ contains all expected symbols."""
    expected_symbols = [
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
    
    for symbol in expected_symbols:
        assert symbol in common.__all__
