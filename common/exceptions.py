"""
Common exceptions for the BuloCloud Sentinel project.

This module provides a set of common exceptions that can be used across all modules.
"""

from __future__ import annotations

from typing import Any, Dict, Optional


class SentinelError(Exception):
    """Base exception for all BuloCloud Sentinel errors."""

    def __init__(
        self, message: str, code: Optional[str] = None, details: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Initialize the exception.

        Args:
            message: The error message
            code: The error code (optional)
            details: Additional error details (optional)
        """
        self.message = message
        self.code = code
        self.details = details or {}
        super().__init__(message)


class ConfigurationError(SentinelError):
    """Exception raised for configuration errors."""

    def __init__(
        self, message: str, config_key: Optional[str] = None, details: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Initialize the exception.

        Args:
            message: The error message
            config_key: The configuration key that caused the error (optional)
            details: Additional error details (optional)
        """
        details = details or {}
        if config_key:
            details["config_key"] = config_key
        super().__init__(message, code="CONFIGURATION_ERROR", details=details)


class ValidationError(SentinelError):
    """Exception raised for validation errors."""

    def __init__(
        self, message: str, field: Optional[str] = None, details: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Initialize the exception.

        Args:
            message: The error message
            field: The field that failed validation (optional)
            details: Additional error details (optional)
        """
        details = details or {}
        if field:
            details["field"] = field
        super().__init__(message, code="VALIDATION_ERROR", details=details)


class AuthenticationError(SentinelError):
    """Exception raised for authentication errors."""

    def __init__(
        self, message: str, user_id: Optional[str] = None, details: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Initialize the exception.

        Args:
            message: The error message
            user_id: The user ID that failed authentication (optional)
            details: Additional error details (optional)
        """
        details = details or {}
        if user_id:
            details["user_id"] = user_id
        super().__init__(message, code="AUTHENTICATION_ERROR", details=details)


class AuthorizationError(SentinelError):
    """Exception raised for authorization errors."""

    def __init__(
        self, message: str, user_id: Optional[str] = None, resource: Optional[str] = None,
        action: Optional[str] = None, details: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Initialize the exception.

        Args:
            message: The error message
            user_id: The user ID that failed authorization (optional)
            resource: The resource that was accessed (optional)
            action: The action that was attempted (optional)
            details: Additional error details (optional)
        """
        details = details or {}
        if user_id:
            details["user_id"] = user_id
        if resource:
            details["resource"] = resource
        if action:
            details["action"] = action
        super().__init__(message, code="AUTHORIZATION_ERROR", details=details)


class ResourceNotFoundError(SentinelError):
    """Exception raised when a resource is not found."""

    def __init__(
        self, message: str, resource_type: Optional[str] = None,
        resource_id: Optional[str] = None, details: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Initialize the exception.

        Args:
            message: The error message
            resource_type: The type of resource that was not found (optional)
            resource_id: The ID of the resource that was not found (optional)
            details: Additional error details (optional)
        """
        details = details or {}
        if resource_type:
            details["resource_type"] = resource_type
        if resource_id:
            details["resource_id"] = resource_id
        super().__init__(message, code="RESOURCE_NOT_FOUND", details=details)


class ServiceUnavailableError(SentinelError):
    """Exception raised when a service is unavailable."""

    def __init__(
        self, message: str, service_name: Optional[str] = None, details: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Initialize the exception.

        Args:
            message: The error message
            service_name: The name of the service that is unavailable (optional)
            details: Additional error details (optional)
        """
        details = details or {}
        if service_name:
            details["service_name"] = service_name
        super().__init__(message, code="SERVICE_UNAVAILABLE", details=details)


class RateLimitError(SentinelError):
    """Exception raised when a rate limit is exceeded."""

    def __init__(
        self, message: str, limit: Optional[int] = None, reset_time: Optional[int] = None,
        details: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Initialize the exception.

        Args:
            message: The error message
            limit: The rate limit that was exceeded (optional)
            reset_time: The time when the rate limit will reset (optional)
            details: Additional error details (optional)
        """
        details = details or {}
        if limit:
            details["limit"] = limit
        if reset_time:
            details["reset_time"] = reset_time
        super().__init__(message, code="RATE_LIMIT_EXCEEDED", details=details)
