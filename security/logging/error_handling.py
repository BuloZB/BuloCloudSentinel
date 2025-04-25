"""
Error handling utilities for Bulo.Cloud Sentinel.

This module provides functions for handling errors and exceptions
in a secure and consistent way.
"""

import sys
import traceback
from typing import Any, Callable, Dict, List, Optional, Type, Union
from functools import wraps

from fastapi import FastAPI, Request, Response, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException

from .secure_logging import SecureLogger, get_secure_logger


# Default error messages
DEFAULT_ERROR_MESSAGES = {
    400: "Bad request",
    401: "Unauthorized",
    403: "Forbidden",
    404: "Not found",
    405: "Method not allowed",
    408: "Request timeout",
    409: "Conflict",
    422: "Validation error",
    429: "Too many requests",
    500: "Internal server error",
    501: "Not implemented",
    502: "Bad gateway",
    503: "Service unavailable",
    504: "Gateway timeout"
}


class ErrorHandler:
    """
    Handler for application errors and exceptions.
    """
    
    def __init__(
        self,
        app: Optional[FastAPI] = None,
        logger: Optional[SecureLogger] = None,
        error_messages: Optional[Dict[int, str]] = None,
        include_exception_details: bool = False,
        log_all_errors: bool = True
    ):
        """
        Initialize the error handler.
        
        Args:
            app: FastAPI application
            logger: Logger to use
            error_messages: Custom error messages
            include_exception_details: Whether to include exception details in responses
            log_all_errors: Whether to log all errors
        """
        self.app = app
        self.logger = logger or get_secure_logger("error_handler")
        self.error_messages = error_messages or DEFAULT_ERROR_MESSAGES
        self.include_exception_details = include_exception_details
        self.log_all_errors = log_all_errors
        
        # Register exception handlers if app is provided
        if app:
            self.register_exception_handlers(app)
    
    def register_exception_handlers(self, app: FastAPI):
        """
        Register exception handlers with a FastAPI application.
        
        Args:
            app: FastAPI application
        """
        # Register handler for RequestValidationError
        app.add_exception_handler(
            RequestValidationError,
            self.validation_exception_handler
        )
        
        # Register handler for HTTPException
        app.add_exception_handler(
            StarletteHTTPException,
            self.http_exception_handler
        )
        
        # Register handler for all other exceptions
        app.add_exception_handler(
            Exception,
            self.generic_exception_handler
        )
    
    async def validation_exception_handler(
        self,
        request: Request,
        exc: RequestValidationError
    ) -> JSONResponse:
        """
        Handle validation exceptions.
        
        Args:
            request: Request that caused the exception
            exc: The exception
            
        Returns:
            JSON response with error details
        """
        # Log the error
        if self.log_all_errors:
            self.logger.error(
                "Validation error",
                {
                    "path": request.url.path,
                    "method": request.method,
                    "client": request.client.host if request.client else None,
                    "errors": exc.errors()
                }
            )
        
        # Create error response
        error_response = {
            "status": "error",
            "code": status.HTTP_422_UNPROCESSABLE_ENTITY,
            "message": self.error_messages.get(422, "Validation error"),
            "errors": exc.errors()
        }
        
        return JSONResponse(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            content=error_response
        )
    
    async def http_exception_handler(
        self,
        request: Request,
        exc: StarletteHTTPException
    ) -> JSONResponse:
        """
        Handle HTTP exceptions.
        
        Args:
            request: Request that caused the exception
            exc: The exception
            
        Returns:
            JSON response with error details
        """
        # Log the error for 5xx status codes
        if self.log_all_errors or exc.status_code >= 500:
            self.logger.error(
                f"HTTP error {exc.status_code}",
                {
                    "path": request.url.path,
                    "method": request.method,
                    "client": request.client.host if request.client else None,
                    "status_code": exc.status_code,
                    "detail": exc.detail
                }
            )
        
        # Create error response
        error_response = {
            "status": "error",
            "code": exc.status_code,
            "message": self.error_messages.get(exc.status_code, str(exc.detail))
        }
        
        # Include exception details if configured
        if self.include_exception_details:
            error_response["detail"] = exc.detail
        
        return JSONResponse(
            status_code=exc.status_code,
            content=error_response,
            headers=getattr(exc, "headers", None)
        )
    
    async def generic_exception_handler(
        self,
        request: Request,
        exc: Exception
    ) -> JSONResponse:
        """
        Handle generic exceptions.
        
        Args:
            request: Request that caused the exception
            exc: The exception
            
        Returns:
            JSON response with error details
        """
        # Log the error
        self.logger.error(
            f"Unhandled exception: {type(exc).__name__}",
            {
                "path": request.url.path,
                "method": request.method,
                "client": request.client.host if request.client else None,
                "exception": str(exc),
                "traceback": traceback.format_exc()
            },
            exc_info=True
        )
        
        # Create error response
        error_response = {
            "status": "error",
            "code": status.HTTP_500_INTERNAL_SERVER_ERROR,
            "message": self.error_messages.get(500, "Internal server error")
        }
        
        # Include exception details if configured
        if self.include_exception_details:
            error_response["detail"] = str(exc)
            error_response["exception_type"] = type(exc).__name__
        
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content=error_response
        )
    
    def handle_exception(
        self,
        exc: Exception,
        status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR,
        log_level: str = "error",
        include_traceback: bool = True
    ) -> Dict[str, Any]:
        """
        Handle an exception and return an error response.
        
        Args:
            exc: The exception
            status_code: HTTP status code
            log_level: Log level
            include_traceback: Whether to include traceback in logs
            
        Returns:
            Error response dictionary
        """
        # Log the error
        log_method = getattr(self.logger, log_level, self.logger.error)
        
        log_data = {
            "exception": str(exc),
            "exception_type": type(exc).__name__
        }
        
        if include_traceback:
            log_data["traceback"] = traceback.format_exc()
        
        log_method(
            f"Exception: {type(exc).__name__}",
            log_data,
            exc_info=include_traceback
        )
        
        # Create error response
        error_response = {
            "status": "error",
            "code": status_code,
            "message": self.error_messages.get(status_code, "An error occurred")
        }
        
        # Include exception details if configured
        if self.include_exception_details:
            error_response["detail"] = str(exc)
            error_response["exception_type"] = type(exc).__name__
        
        return error_response


def exception_handler(
    exc_type: Type[Exception],
    status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR,
    log_level: str = "error",
    include_traceback: bool = True,
    logger: Optional[SecureLogger] = None
):
    """
    Decorator for handling exceptions in functions.
    
    Args:
        exc_type: Exception type to catch
        status_code: HTTP status code
        log_level: Log level
        include_traceback: Whether to include traceback in logs
        logger: Logger to use
        
    Returns:
        Decorator function
    """
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except exc_type as e:
                # Get or create logger
                log = logger or get_secure_logger(func.__module__)
                
                # Log the error
                log_method = getattr(log, log_level, log.error)
                
                log_data = {
                    "function": func.__name__,
                    "args": args,
                    "kwargs": kwargs,
                    "exception": str(e),
                    "exception_type": type(e).__name__
                }
                
                if include_traceback:
                    log_data["traceback"] = traceback.format_exc()
                
                log_method(
                    f"Exception in {func.__name__}: {type(e).__name__}",
                    log_data,
                    exc_info=include_traceback
                )
                
                # Re-raise the exception
                raise
        
        return wrapper
    
    return decorator


def async_exception_handler(
    exc_type: Type[Exception],
    status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR,
    log_level: str = "error",
    include_traceback: bool = True,
    logger: Optional[SecureLogger] = None
):
    """
    Decorator for handling exceptions in async functions.
    
    Args:
        exc_type: Exception type to catch
        status_code: HTTP status code
        log_level: Log level
        include_traceback: Whether to include traceback in logs
        logger: Logger to use
        
    Returns:
        Decorator function
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            try:
                return await func(*args, **kwargs)
            except exc_type as e:
                # Get or create logger
                log = logger or get_secure_logger(func.__module__)
                
                # Log the error
                log_method = getattr(log, log_level, log.error)
                
                log_data = {
                    "function": func.__name__,
                    "args": args,
                    "kwargs": kwargs,
                    "exception": str(e),
                    "exception_type": type(e).__name__
                }
                
                if include_traceback:
                    log_data["traceback"] = traceback.format_exc()
                
                log_method(
                    f"Exception in {func.__name__}: {type(e).__name__}",
                    log_data,
                    exc_info=include_traceback
                )
                
                # Re-raise the exception
                raise
        
        return wrapper
    
    return decorator
