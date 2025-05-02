"""
Error handling utilities for Bulo.Cloud Sentinel.

This package provides functions for secure error handling and exception management.
"""

from .secure_error_handler import (
    ErrorType,
    ErrorResponse,
    create_error_response,
    handle_http_exception,
    handle_validation_exception,
    handle_internal_exception,
    configure_error_handlers,
    configure_custom_exception_handlers,
    SecurityException,
    AuthenticationException,
    AuthorizationException,
    RateLimitException,
    ValidationException,
)

__all__ = [
    "ErrorType",
    "ErrorResponse",
    "create_error_response",
    "handle_http_exception",
    "handle_validation_exception",
    "handle_internal_exception",
    "configure_error_handlers",
    "configure_custom_exception_handlers",
    "SecurityException",
    "AuthenticationException",
    "AuthorizationException",
    "RateLimitException",
    "ValidationException",
]
