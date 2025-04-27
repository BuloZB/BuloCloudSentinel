"""
Input validation middleware for Bulo.Cloud Sentinel.

This module provides middleware for validating incoming requests
against defined schemas.
"""

import logging
from typing import Dict, List, Optional, Any, Callable, Type, Union
import json

from fastapi import FastAPI, Request, Response, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ValidationError
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from ..logging.secure_logging import get_secure_logger


class InputValidationMiddleware(BaseHTTPMiddleware):
    """
    Middleware for validating incoming requests against defined schemas.
    """
    
    def __init__(
        self,
        app: ASGIApp,
        schemas: Dict[str, Type[BaseModel]],
        exclude_paths: Optional[List[str]] = None,
        error_handler: Optional[Callable[[Request, ValidationError], Response]] = None
    ):
        """
        Initialize the input validation middleware.
        
        Args:
            app: ASGI application
            schemas: Dictionary mapping paths to Pydantic models
            exclude_paths: Paths to exclude from validation
            error_handler: Custom error handler
        """
        super().__init__(app)
        self.schemas = schemas
        self.exclude_paths = exclude_paths or []
        self.error_handler = error_handler
        self.logger = get_secure_logger("input_validation")
        
        # Log configuration
        self.logger.info(
            "Configuring input validation middleware",
            {
                "schemas": {path: schema.__name__ for path, schema in self.schemas.items()},
                "exclude_paths": self.exclude_paths
            }
        )
    
    async def dispatch(self, request: Request, call_next):
        """
        Process a request.
        
        Args:
            request: Request object
            call_next: Function to call next middleware
            
        Returns:
            Response
        """
        # Skip excluded paths
        if any(request.url.path.startswith(path) for path in self.exclude_paths):
            return await call_next(request)
        
        # Skip methods that don't have a body
        if request.method in ["GET", "HEAD", "OPTIONS", "DELETE"]:
            return await call_next(request)
        
        # Find matching schema
        schema = None
        for path, model in self.schemas.items():
            if request.url.path.startswith(path):
                schema = model
                break
        
        # Skip if no schema found
        if not schema:
            return await call_next(request)
        
        # Validate request body
        try:
            # Read request body
            body = await request.body()
            if not body:
                return await call_next(request)
            
            # Parse JSON
            try:
                data = json.loads(body)
            except json.JSONDecodeError as e:
                self.logger.warning(
                    "Invalid JSON in request body",
                    {
                        "path": request.url.path,
                        "method": request.method,
                        "error": str(e)
                    }
                )
                return JSONResponse(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    content={"detail": "Invalid JSON in request body"}
                )
            
            # Validate against schema
            try:
                schema(**data)
            except ValidationError as e:
                self.logger.warning(
                    "Validation error",
                    {
                        "path": request.url.path,
                        "method": request.method,
                        "errors": e.errors()
                    }
                )
                
                # Use custom error handler if provided
                if self.error_handler:
                    return self.error_handler(request, e)
                
                # Default error response
                return JSONResponse(
                    status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                    content={"detail": e.errors()}
                )
            
            # Create new request with validated body
            async def receive():
                return {"type": "http.request", "body": body}
            
            request._receive = receive
            
            # Process request
            return await call_next(request)
            
        except Exception as e:
            self.logger.error(
                "Error validating request",
                {
                    "path": request.url.path,
                    "method": request.method,
                    "error": str(e)
                }
            )
            return JSONResponse(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                content={"detail": "Internal server error"}
            )


def add_input_validation(
    app: FastAPI,
    schemas: Dict[str, Type[BaseModel]],
    exclude_paths: Optional[List[str]] = None,
    error_handler: Optional[Callable[[Request, ValidationError], Response]] = None
):
    """
    Add input validation middleware to a FastAPI application.
    
    Args:
        app: FastAPI application
        schemas: Dictionary mapping paths to Pydantic models
        exclude_paths: Paths to exclude from validation
        error_handler: Custom error handler
    """
    app.add_middleware(
        InputValidationMiddleware,
        schemas=schemas,
        exclude_paths=exclude_paths,
        error_handler=error_handler
    )
