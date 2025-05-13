"""
Tests for the common exceptions module.

This module contains tests for the common exceptions module.
"""

from __future__ import annotations

import pytest

from common.exceptions import (
    SentinelError,
    ConfigurationError,
    ValidationError,
    AuthenticationError,
    AuthorizationError,
    ResourceNotFoundError,
    ServiceUnavailableError,
    RateLimitError,
)


def test_sentinel_error() -> None:
    """Test the SentinelError class."""
    # Create a basic error
    error = SentinelError("An error occurred")
    assert str(error) == "An error occurred"
    assert error.message == "An error occurred"
    assert error.code is None
    assert error.details == {}
    
    # Create an error with code and details
    error = SentinelError(
        message="An error occurred",
        code="ERROR_CODE",
        details={"key": "value"},
    )
    assert str(error) == "An error occurred"
    assert error.message == "An error occurred"
    assert error.code == "ERROR_CODE"
    assert error.details == {"key": "value"}


def test_configuration_error() -> None:
    """Test the ConfigurationError class."""
    # Create a basic error
    error = ConfigurationError("Configuration error")
    assert str(error) == "Configuration error"
    assert error.message == "Configuration error"
    assert error.code == "CONFIGURATION_ERROR"
    assert error.details == {}
    
    # Create an error with config_key
    error = ConfigurationError(
        message="Configuration error",
        config_key="database.url",
    )
    assert str(error) == "Configuration error"
    assert error.message == "Configuration error"
    assert error.code == "CONFIGURATION_ERROR"
    assert error.details == {"config_key": "database.url"}
    
    # Create an error with config_key and details
    error = ConfigurationError(
        message="Configuration error",
        config_key="database.url",
        details={"suggestion": "Check your .env file"},
    )
    assert str(error) == "Configuration error"
    assert error.message == "Configuration error"
    assert error.code == "CONFIGURATION_ERROR"
    assert error.details == {
        "config_key": "database.url",
        "suggestion": "Check your .env file",
    }


def test_validation_error() -> None:
    """Test the ValidationError class."""
    # Create a basic error
    error = ValidationError("Validation error")
    assert str(error) == "Validation error"
    assert error.message == "Validation error"
    assert error.code == "VALIDATION_ERROR"
    assert error.details == {}
    
    # Create an error with field
    error = ValidationError(
        message="Validation error",
        field="email",
    )
    assert str(error) == "Validation error"
    assert error.message == "Validation error"
    assert error.code == "VALIDATION_ERROR"
    assert error.details == {"field": "email"}
    
    # Create an error with field and details
    error = ValidationError(
        message="Validation error",
        field="email",
        details={"expected": "valid email address"},
    )
    assert str(error) == "Validation error"
    assert error.message == "Validation error"
    assert error.code == "VALIDATION_ERROR"
    assert error.details == {
        "field": "email",
        "expected": "valid email address",
    }


def test_authentication_error() -> None:
    """Test the AuthenticationError class."""
    # Create a basic error
    error = AuthenticationError("Authentication error")
    assert str(error) == "Authentication error"
    assert error.message == "Authentication error"
    assert error.code == "AUTHENTICATION_ERROR"
    assert error.details == {}
    
    # Create an error with user_id
    error = AuthenticationError(
        message="Authentication error",
        user_id="user123",
    )
    assert str(error) == "Authentication error"
    assert error.message == "Authentication error"
    assert error.code == "AUTHENTICATION_ERROR"
    assert error.details == {"user_id": "user123"}


def test_authorization_error() -> None:
    """Test the AuthorizationError class."""
    # Create a basic error
    error = AuthorizationError("Authorization error")
    assert str(error) == "Authorization error"
    assert error.message == "Authorization error"
    assert error.code == "AUTHORIZATION_ERROR"
    assert error.details == {}
    
    # Create an error with all fields
    error = AuthorizationError(
        message="Authorization error",
        user_id="user123",
        resource="document",
        action="edit",
    )
    assert str(error) == "Authorization error"
    assert error.message == "Authorization error"
    assert error.code == "AUTHORIZATION_ERROR"
    assert error.details == {
        "user_id": "user123",
        "resource": "document",
        "action": "edit",
    }


def test_resource_not_found_error() -> None:
    """Test the ResourceNotFoundError class."""
    # Create a basic error
    error = ResourceNotFoundError("Resource not found")
    assert str(error) == "Resource not found"
    assert error.message == "Resource not found"
    assert error.code == "RESOURCE_NOT_FOUND"
    assert error.details == {}
    
    # Create an error with resource_type and resource_id
    error = ResourceNotFoundError(
        message="Resource not found",
        resource_type="User",
        resource_id="user123",
    )
    assert str(error) == "Resource not found"
    assert error.message == "Resource not found"
    assert error.code == "RESOURCE_NOT_FOUND"
    assert error.details == {
        "resource_type": "User",
        "resource_id": "user123",
    }


def test_service_unavailable_error() -> None:
    """Test the ServiceUnavailableError class."""
    # Create a basic error
    error = ServiceUnavailableError("Service unavailable")
    assert str(error) == "Service unavailable"
    assert error.message == "Service unavailable"
    assert error.code == "SERVICE_UNAVAILABLE"
    assert error.details == {}
    
    # Create an error with service_name
    error = ServiceUnavailableError(
        message="Service unavailable",
        service_name="database",
    )
    assert str(error) == "Service unavailable"
    assert error.message == "Service unavailable"
    assert error.code == "SERVICE_UNAVAILABLE"
    assert error.details == {"service_name": "database"}


def test_rate_limit_error() -> None:
    """Test the RateLimitError class."""
    # Create a basic error
    error = RateLimitError("Rate limit exceeded")
    assert str(error) == "Rate limit exceeded"
    assert error.message == "Rate limit exceeded"
    assert error.code == "RATE_LIMIT_EXCEEDED"
    assert error.details == {}
    
    # Create an error with limit and reset_time
    error = RateLimitError(
        message="Rate limit exceeded",
        limit=100,
        reset_time=1609459200,
    )
    assert str(error) == "Rate limit exceeded"
    assert error.message == "Rate limit exceeded"
    assert error.code == "RATE_LIMIT_EXCEEDED"
    assert error.details == {
        "limit": 100,
        "reset_time": 1609459200,
    }
