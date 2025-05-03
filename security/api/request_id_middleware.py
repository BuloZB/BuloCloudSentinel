"""
Request ID middleware for Bulo.Cloud Sentinel.

This module provides middleware to add a request ID to all requests, which can be used
for tracking and correlation of logs and errors.
"""

import uuid
from typing import Optional
from fastapi import FastAPI, Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from security.logging.secure_logging import get_secure_logger

# Set up logging
logger = get_secure_logger("request_id_middleware")

class RequestIDMiddleware(BaseHTTPMiddleware):
    """
    Middleware to add a request ID to all requests.
    """
    
    def __init__(
        self,
        app: ASGIApp,
        header_name: str = "X-Request-ID",
        generate_if_missing: bool = True,
        include_in_response: bool = True
    ):
        """
        Initialize the request ID middleware.
        
        Args:
            app: ASGI application
            header_name: Name of the request ID header
            generate_if_missing: Whether to generate a request ID if it's missing
            include_in_response: Whether to include the request ID in the response
        """
        super().__init__(app)
        self.header_name = header_name
        self.generate_if_missing = generate_if_missing
        self.include_in_response = include_in_response
        
        # Log configuration
        logger.info(
            "Configuring request ID middleware",
            extra={
                "header_name": self.header_name,
                "generate_if_missing": self.generate_if_missing,
                "include_in_response": self.include_in_response
            }
        )
    
    async def dispatch(self, request: Request, call_next) -> Response:
        """
        Dispatch the request and add a request ID.
        
        Args:
            request: Request object
            call_next: Next middleware or route handler
            
        Returns:
            Response
        """
        # Get the request ID from the header
        request_id = request.headers.get(self.header_name)
        
        # Generate a request ID if it's missing and generation is enabled
        if not request_id and self.generate_if_missing:
            request_id = str(uuid.uuid4())
        
        # Store the request ID in the request state
        if request_id:
            request.state.request_id = request_id
        
        # Process the request
        response = await call_next(request)
        
        # Add the request ID to the response if enabled
        if self.include_in_response and request_id:
            response.headers[self.header_name] = request_id
        
        return response

def add_request_id_middleware(
    app: FastAPI,
    header_name: str = "X-Request-ID",
    generate_if_missing: bool = True,
    include_in_response: bool = True
) -> None:
    """
    Add request ID middleware to a FastAPI application.
    
    Args:
        app: FastAPI application
        header_name: Name of the request ID header
        generate_if_missing: Whether to generate a request ID if it's missing
        include_in_response: Whether to include the request ID in the response
    """
    # Add request ID middleware
    app.add_middleware(
        RequestIDMiddleware,
        header_name=header_name,
        generate_if_missing=generate_if_missing,
        include_in_response=include_in_response
    )
    
    logger.info("Added request ID middleware to FastAPI application")

def get_request_id(request: Request, header_name: str = "X-Request-ID") -> Optional[str]:
    """
    Get the request ID from a request.
    
    Args:
        request: FastAPI request object
        header_name: Name of the request ID header
        
    Returns:
        Request ID if available, None otherwise
    """
    # Try to get from state
    if hasattr(request.state, "request_id"):
        return request.state.request_id
    
    # Try to get from headers
    return request.headers.get(header_name)
