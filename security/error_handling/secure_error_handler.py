"""
Secure error handling module for Bulo.Cloud Sentinel.

This module provides a standardized approach to error handling across all components
of the Bulo.Cloud Sentinel platform, ensuring that sensitive information is not leaked
in error messages.
"""

import logging
import traceback
import sys
from typing import Any, Dict, List, Optional, Union, Type
from fastapi import FastAPI, Request, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from pydantic import ValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException

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
        code: Optional[str] = None
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
        """
        self.error_type = error_type
        self.message = message
        self.status_code = status_code
        self.details = details or []
        self.request_id = request_id
        self.code = code
    
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
                "status_code": self.status_code
            }
        }
        
        if self.details:
            response["error"]["details"] = self.details
        
        if self.request_id:
            response["error"]["request_id"] = self.request_id
        
        if self.code:
            response["error"]["code"] = self.code
        
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

def get_request_id(request: Request) -> Optional[str]:
    """
    Get the request ID from the request.
    
    Args:
        request: FastAPI request object
        
    Returns:
        Request ID if available, None otherwise
    """
    if hasattr(request.state, "request_id"):
        return request.state.request_id
    
    # Try to get from headers
    return request.headers.get("X-Request-ID")

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
    error_type = ErrorType.BAD_REQUEST
    
    if exc.status_code == status.HTTP_401_UNAUTHORIZED:
        error_type = ErrorType.AUTHENTICATION
    elif exc.status_code == status.HTTP_403_FORBIDDEN:
        error_type = ErrorType.AUTHORIZATION
    elif exc.status_code == status.HTTP_404_NOT_FOUND:
        error_type = ErrorType.RESOURCE_NOT_FOUND
    elif exc.status_code == status.HTTP_409_CONFLICT:
        error_type = ErrorType.RESOURCE_CONFLICT
    elif exc.status_code == status.HTTP_429_TOO_MANY_REQUESTS:
        error_type = ErrorType.RATE_LIMIT
    elif exc.status_code == status.HTTP_500_INTERNAL_SERVER_ERROR:
        error_type = ErrorType.INTERNAL_SERVER
    elif exc.status_code == status.HTTP_503_SERVICE_UNAVAILABLE:
        error_type = ErrorType.SERVICE_UNAVAILABLE
    
    # Get request ID
    request_id = get_request_id(request)
    
    # Log the error
    logger.error(
        f"HTTP exception: {exc.status_code} - {exc.detail}",
        extra={
            "request_id": request_id,
            "status_code": exc.status_code,
            "error_type": error_type,
            "path": request.url.path,
            "method": request.method
        }
    )
    
    # Create error response
    return create_error_response(
        error_type=error_type,
        message=str(exc.detail),
        status_code=exc.status_code,
        request_id=request_id
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
        error_detail = {
            "loc": error.get("loc", []),
            "msg": error.get("msg", ""),
            "type": error.get("type", "")
        }
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
        message="Validation error",
        status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
        details=details,
        request_id=request_id
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
    
    # Log the error with full traceback
    logger.error(
        f"Internal server error: {str(exc)}",
        extra={
            "request_id": request_id,
            "path": request.url.path,
            "method": request.method,
            "traceback": traceback_str,
            "exception_type": exc.__class__.__name__
        }
    )
    
    # Create error response with sanitized message
    return create_error_response(
        error_type=ErrorType.INTERNAL_SERVER,
        message="An internal server error occurred",
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        request_id=request_id
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
