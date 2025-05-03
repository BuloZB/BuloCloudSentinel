"""
Secure error handling module for Bulo.Cloud Sentinel.

This module provides a standardized approach to error handling across all components
of the Bulo.Cloud Sentinel platform, ensuring that sensitive information is not leaked
in error messages.
"""

import logging
import traceback
import sys
import re
import uuid
import json
from datetime import datetime
from typing import Any, Dict, List, Optional, Union, Type, Callable
from fastapi import FastAPI, Request, status, Response
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.base import BaseHTTPMiddleware
from pydantic import ValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from starlette.types import ASGIApp

from security.logging.secure_logging import get_secure_logger

# Set up logging
logger = get_secure_logger("error_handler")

# Define error types
class ErrorType:
    """Error type constants."""
    AUTHENTICATION = "authentication_error"
    AUTHORIZATION = "authorization_error"
    VALIDATION = "validation_error"
    RESOURCE_NOT_FOUND = "resource_not_found"
    RESOURCE_CONFLICT = "resource_conflict"
    RATE_LIMIT = "rate_limit_error"
    INTERNAL_SERVER = "internal_server_error"
    BAD_REQUEST = "bad_request"
    SERVICE_UNAVAILABLE = "service_unavailable"
    DATABASE = "database_error"
    EXTERNAL_SERVICE = "external_service_error"
    SECURITY = "security_error"
    INPUT_VALIDATION = "input_validation_error"
    BUSINESS_LOGIC = "business_logic_error"
    CONFIGURATION = "configuration_error"

    @classmethod
    def from_status_code(cls, status_code: int) -> str:
        """
        Get error type from status code.

        Args:
            status_code: HTTP status code

        Returns:
            Error type
        """
        if status_code == status.HTTP_400_BAD_REQUEST:
            return cls.BAD_REQUEST
        elif status_code == status.HTTP_401_UNAUTHORIZED:
            return cls.AUTHENTICATION
        elif status_code == status.HTTP_403_FORBIDDEN:
            return cls.AUTHORIZATION
        elif status_code == status.HTTP_404_NOT_FOUND:
            return cls.RESOURCE_NOT_FOUND
        elif status_code == status.HTTP_409_CONFLICT:
            return cls.RESOURCE_CONFLICT
        elif status_code == status.HTTP_422_UNPROCESSABLE_ENTITY:
            return cls.VALIDATION
        elif status_code == status.HTTP_429_TOO_MANY_REQUESTS:
            return cls.RATE_LIMIT
        elif status_code == status.HTTP_503_SERVICE_UNAVAILABLE:
            return cls.SERVICE_UNAVAILABLE
        elif status_code >= 500:
            return cls.INTERNAL_SERVER
        else:
            return cls.BAD_REQUEST

# Define error response model
class ErrorResponse:
    """Error response model."""

    def __init__(
        self,
        error_type: str,
        message: str,
        status_code: int,
        details: Optional[List[Dict[str, Any]]] = None,
        request_id: Optional[str] = None,
        code: Optional[str] = None,
        path: Optional[str] = None,
        timestamp: Optional[str] = None,
        instance: Optional[str] = None,
        help_url: Optional[str] = None
    ):
        """
        Initialize the error response.

        Args:
            error_type: Type of error
            message: Error message
            status_code: HTTP status code
            details: Optional error details
            request_id: Optional request ID for tracking
            code: Optional error code
            path: Optional path where the error occurred
            timestamp: Optional timestamp of the error
            instance: Optional instance identifier
            help_url: Optional URL for help documentation
        """
        self.error_type = error_type
        self.message = self._sanitize_message(message)
        self.status_code = status_code
        self.details = details or []
        self.request_id = request_id
        self.code = code
        self.path = path
        self.timestamp = timestamp or datetime.utcnow().isoformat()
        self.instance = instance
        self.help_url = help_url

    def _sanitize_message(self, message: str) -> str:
        """
        Sanitize error message to remove sensitive information.

        Args:
            message: Error message

        Returns:
            Sanitized error message
        """
        # Define patterns for sensitive information
        patterns = [
            # Database connection strings
            r'(mongodb|mysql|postgres|postgresql|sqlite|sqlserver|oracle|jdbc|odbc):[^\"\']*',
            # API keys and tokens
            r'(api[_-]?key|token|secret|password|pwd|auth)[^\w]?[:=][^\w]?[\'\"][^\'\"]{5,}[\'\"]',
            # Email addresses
            r'[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}',
            # IP addresses
            r'\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b',
            # Stack traces
            r'at\s+[\w.]+\([\w.:]+\)',
            r'File\s+"[^"]+",\s+line\s+\d+',
            # SQL queries
            r'(SELECT|INSERT|UPDATE|DELETE|CREATE|ALTER|DROP).*FROM.*',
            # Exception class names
            r'Exception:\s+.*',
            r'Error:\s+.*',
            # Paths and filenames
            r'[A-Za-z]:\\[^:]*',
            r'/[a-zA-Z0-9_.-/]+',
        ]

        # Replace sensitive information with placeholders
        sanitized = message
        for pattern in patterns:
            sanitized = re.sub(pattern, '[REDACTED]', sanitized, flags=re.IGNORECASE)

        return sanitized

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the error response to a dictionary.

        Returns:
            Dictionary representation of the error response
        """
        response = {
            "error": {
                "type": self.error_type,
                "message": self.message,
                "status_code": self.status_code,
                "timestamp": self.timestamp
            }
        }

        if self.details:
            response["error"]["details"] = self.details

        if self.request_id:
            response["error"]["request_id"] = self.request_id

        if self.code:
            response["error"]["code"] = self.code

        if self.path:
            response["error"]["path"] = self.path

        if self.instance:
            response["error"]["instance"] = self.instance

        if self.help_url:
            response["error"]["help_url"] = self.help_url

        return response

# Error handler functions
def create_error_response(
    error_type: str,
    message: str,
    status_code: int,
    details: Optional[List[Dict[str, Any]]] = None,
    request_id: Optional[str] = None,
    code: Optional[str] = None
) -> JSONResponse:
    """
    Create an error response.

    Args:
        error_type: Type of error
        message: Error message
        status_code: HTTP status code
        details: Optional error details
        request_id: Optional request ID for tracking
        code: Optional error code

    Returns:
        JSON response with error details
    """
    error_response = ErrorResponse(
        error_type=error_type,
        message=message,
        status_code=status_code,
        details=details,
        request_id=request_id,
        code=code
    )

    return JSONResponse(
        status_code=status_code,
        content=error_response.to_dict()
    )

def get_request_id(request: Request) -> str:
    """
    Get the request ID from the request or generate a new one.

    Args:
        request: FastAPI request object

    Returns:
        Request ID
    """
    # Try to get from state
    if hasattr(request.state, "request_id"):
        return request.state.request_id

    # Try to get from headers
    request_id = request.headers.get("X-Request-ID")

    # Generate a new request ID if not found
    if not request_id:
        request_id = str(uuid.uuid4())

        # Store in request state for future use
        if not hasattr(request.state, "request_id"):
            request.state.request_id = request_id

    return request_id

def handle_http_exception(request: Request, exc: StarletteHTTPException) -> JSONResponse:
    """
    Handle HTTP exceptions.

    Args:
        request: FastAPI request object
        exc: HTTP exception

    Returns:
        JSON response with error details
    """
    # Map status code to error type
    error_type = ErrorType.from_status_code(exc.status_code)

    # Get request ID
    request_id = get_request_id(request)

    # Get error details from headers if available
    details = None
    if hasattr(exc, "headers") and exc.headers:
        error_details = exc.headers.get("X-Error-Details")
        if error_details:
            try:
                details = json.loads(error_details)
            except json.JSONDecodeError:
                details = [{"detail": error_details}]

    # Log the error
    logger.error(
        f"HTTP exception: {exc.status_code} - {exc.detail}",
        extra={
            "request_id": request_id,
            "status_code": exc.status_code,
            "error_type": error_type,
            "path": request.url.path,
            "method": request.method,
            "details": details
        }
    )

    # Create error response
    return create_error_response(
        error_type=error_type,
        message=str(exc.detail),
        status_code=exc.status_code,
        details=details,
        request_id=request_id,
        path=request.url.path
    )

def handle_validation_exception(request: Request, exc: RequestValidationError) -> JSONResponse:
    """
    Handle validation exceptions.

    Args:
        request: FastAPI request object
        exc: Validation exception

    Returns:
        JSON response with error details
    """
    # Get request ID
    request_id = get_request_id(request)

    # Extract error details
    details = []
    for error in exc.errors():
        # Sanitize error message
        msg = error.get("msg", "")
        if msg:
            # Remove any sensitive information from the error message
            msg = re.sub(r'(password|token|secret|key)=[\w\d]+', r'\1=[REDACTED]', msg)

        error_detail = {
            "loc": error.get("loc", []),
            "msg": msg,
            "type": error.get("type", "")
        }

        # Add input value for non-sensitive fields
        if "loc" in error and error["loc"]:
            field_name = error["loc"][-1] if isinstance(error["loc"], (list, tuple)) else error["loc"]
            if not any(sensitive in str(field_name).lower() for sensitive in ["password", "token", "secret", "key", "auth"]):
                error_detail["input"] = error.get("input")

        details.append(error_detail)

    # Log the error
    logger.warning(
        f"Validation error: {len(details)} errors",
        extra={
            "request_id": request_id,
            "path": request.url.path,
            "method": request.method,
            "errors": details
        }
    )

    # Create error response
    return create_error_response(
        error_type=ErrorType.VALIDATION,
        message="Request validation error",
        status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
        details=details,
        request_id=request_id,
        path=request.url.path,
        code="VALIDATION_ERROR"
    )

def handle_internal_exception(request: Request, exc: Exception) -> JSONResponse:
    """
    Handle internal server exceptions.

    Args:
        request: FastAPI request object
        exc: Exception

    Returns:
        JSON response with error details
    """
    # Get request ID
    request_id = get_request_id(request)

    # Get exception details
    exc_type, exc_value, exc_traceback = sys.exc_info()

    # Format traceback
    traceback_str = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))

    # Generate error code based on exception type
    error_code = f"INTERNAL_ERROR_{exc.__class__.__name__.upper()}"

    # Determine if this is a known exception type
    error_type = ErrorType.INTERNAL_SERVER
    if "database" in exc.__class__.__name__.lower() or "sql" in exc.__class__.__name__.lower():
        error_type = ErrorType.DATABASE
    elif "timeout" in exc.__class__.__name__.lower() or "connection" in exc.__class__.__name__.lower():
        error_type = ErrorType.EXTERNAL_SERVICE
    elif "validation" in exc.__class__.__name__.lower() or "invalid" in exc.__class__.__name__.lower():
        error_type = ErrorType.VALIDATION
    elif "auth" in exc.__class__.__name__.lower() or "permission" in exc.__class__.__name__.lower():
        error_type = ErrorType.AUTHORIZATION

    # Log the error with full traceback
    logger.error(
        f"Internal server error: {str(exc)}",
        extra={
            "request_id": request_id,
            "path": request.url.path,
            "method": request.method,
            "traceback": traceback_str,
            "exception_type": exc.__class__.__name__,
            "error_type": error_type,
            "error_code": error_code
        }
    )

    # Create error response with sanitized message
    return create_error_response(
        error_type=error_type,
        message="An internal server error occurred",
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        request_id=request_id,
        path=request.url.path,
        code=error_code
    )

def configure_error_handlers(app: FastAPI) -> None:
    """
    Configure error handlers for a FastAPI application.

    Args:
        app: FastAPI application
    """
    # Add exception handlers
    app.add_exception_handler(StarletteHTTPException, handle_http_exception)
    app.add_exception_handler(RequestValidationError, handle_validation_exception)
    app.add_exception_handler(Exception, handle_internal_exception)

    logger.info("Configured secure error handlers")

# Create custom exceptions
class SecurityException(Exception):
    """Base class for security exceptions."""

    def __init__(self, message: str, status_code: int = status.HTTP_400_BAD_REQUEST):
        """
        Initialize the security exception.

        Args:
            message: Error message
            status_code: HTTP status code
        """
        self.message = message
        self.status_code = status_code
        super().__init__(self.message)

class AuthenticationException(SecurityException):
    """Exception raised for authentication errors."""

    def __init__(self, message: str = "Authentication failed"):
        """
        Initialize the authentication exception.

        Args:
            message: Error message
        """
        super().__init__(message, status.HTTP_401_UNAUTHORIZED)

class AuthorizationException(SecurityException):
    """Exception raised for authorization errors."""

    def __init__(self, message: str = "Not authorized to perform this action"):
        """
        Initialize the authorization exception.

        Args:
            message: Error message
        """
        super().__init__(message, status.HTTP_403_FORBIDDEN)

class RateLimitException(SecurityException):
    """Exception raised for rate limit errors."""

    def __init__(self, message: str = "Rate limit exceeded"):
        """
        Initialize the rate limit exception.

        Args:
            message: Error message
        """
        super().__init__(message, status.HTTP_429_TOO_MANY_REQUESTS)

class ValidationException(SecurityException):
    """Exception raised for validation errors."""

    def __init__(self, message: str = "Validation error", details: Optional[List[Dict[str, Any]]] = None):
        """
        Initialize the validation exception.

        Args:
            message: Error message
            details: Validation error details
        """
        self.details = details or []
        super().__init__(message, status.HTTP_422_UNPROCESSABLE_ENTITY)

# Configure exception handlers for custom exceptions
def configure_custom_exception_handlers(app: FastAPI) -> None:
    """
    Configure custom exception handlers for a FastAPI application.

    Args:
        app: FastAPI application
    """
    # Add custom exception handlers
    app.add_exception_handler(
        AuthenticationException,
        lambda request, exc: create_error_response(
            error_type=ErrorType.AUTHENTICATION,
            message=exc.message,
            status_code=exc.status_code,
            request_id=get_request_id(request)
        )
    )

    app.add_exception_handler(
        AuthorizationException,
        lambda request, exc: create_error_response(
            error_type=ErrorType.AUTHORIZATION,
            message=exc.message,
            status_code=exc.status_code,
            request_id=get_request_id(request)
        )
    )

    app.add_exception_handler(
        RateLimitException,
        lambda request, exc: create_error_response(
            error_type=ErrorType.RATE_LIMIT,
            message=exc.message,
            status_code=exc.status_code,
            request_id=get_request_id(request)
        )
    )

    app.add_exception_handler(
        ValidationException,
        lambda request, exc: create_error_response(
            error_type=ErrorType.VALIDATION,
            message=exc.message,
            status_code=exc.status_code,
            details=exc.details,
            request_id=get_request_id(request)
        )
    )

    logger.info("Configured custom exception handlers")
