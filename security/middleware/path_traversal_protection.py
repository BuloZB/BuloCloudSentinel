"""
Path traversal protection middleware for FastAPI.

This middleware adds path traversal protection to HTTP requests.
"""

import os
import re
from typing import Callable, Dict, List, Optional, Set, Any

from fastapi import FastAPI, Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

from security.utils.input_sanitizer import is_path_traversal
from security.utils.input_sanitizer import is_command_injection, is_sql_injection
from security.logging.secure_logging import get_secure_logger

# Configure secure logger
logger = get_secure_logger("path_traversal_protection")

class PathTraversalProtectionMiddleware(BaseHTTPMiddleware):
    """
    Middleware for path traversal protection.
    
    This middleware adds path traversal protection to HTTP requests.
    """
    
    def __init__(
        self,
        app: FastAPI,
        base_path: Optional[str] = None,
        exclude_paths: Optional[List[str]] = None,
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            base_path: Base path for path traversal protection
            exclude_paths: List of paths to exclude from path traversal protection
        """
        super().__init__(app)
        self.base_path = base_path or os.getcwd()
        self.exclude_paths = exclude_paths or []
    
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and add comprehensive path parameter validation.
        
        This middleware checks for path traversal, command injection, and SQL injection
        in path parameters and query parameters.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response
        """
        # Skip protection for excluded paths
        path = request.url.path
        if any(path.startswith(excluded) for excluded in self.exclude_paths):
            return await call_next(request)
        
        client_ip = request.client.host if request.client else "unknown"
        
        # Check query parameters for various injection attacks
        for key, value in request.query_params.items():
            # Skip empty values
            if not value:
                continue
                
            # Check for path traversal
            if is_path_traversal(value):
                logger.warning(
                    f"Path traversal detected in query parameter",
                    {
                        "client_ip": client_ip,
                        "path": request.url.path,
                        "parameter": key,
                        "value": value
                    }
                )
                return JSONResponse(
                    status_code=400,
                    content={"detail": f"Invalid characters detected in query parameter: {key}"}
                )
            
            # Check for command injection
            if is_command_injection(value):
                logger.warning(
                    f"Command injection detected in query parameter",
                    {
                        "client_ip": client_ip,
                        "path": request.url.path,
                        "parameter": key,
                        "value": value
                    }
                )
                return JSONResponse(
                    status_code=400,
                    content={"detail": f"Invalid characters detected in query parameter: {key}"}
                )
            
            # Check for SQL injection
            if is_sql_injection(value):
                logger.warning(
                    f"SQL injection detected in query parameter",
                    {
                        "client_ip": client_ip,
                        "path": request.url.path,
                        "parameter": key,
                        "value": value
                    }
                )
                return JSONResponse(
                    status_code=400,
                    content={"detail": f"Invalid characters detected in query parameter: {key}"}
                )
        
        # Check path parameters for various injection attacks
        for key, value in request.path_params.items():
            # Convert to string if not already
            str_value = str(value)
            
            # Skip empty values
            if not str_value:
                continue
                
            # Check for path traversal
            if is_path_traversal(str_value):
                logger.warning(
                    f"Path traversal detected in path parameter",
                    {
                        "client_ip": client_ip,
                        "path": request.url.path,
                        "parameter": key,
                        "value": str_value
                    }
                )
                return JSONResponse(
                    status_code=400,
                    content={"detail": f"Invalid characters detected in path parameter: {key}"}
                )
            
            # Check for command injection
            if is_command_injection(str_value):
                logger.warning(
                    f"Command injection detected in path parameter",
                    {
                        "client_ip": client_ip,
                        "path": request.url.path,
                        "parameter": key,
                        "value": str_value
                    }
                )
                return JSONResponse(
                    status_code=400,
                    content={"detail": f"Invalid characters detected in path parameter: {key}"}
                )
            
            # Check for SQL injection
            if is_sql_injection(str_value):
                logger.warning(
                    f"SQL injection detected in path parameter",
                    {
                        "client_ip": client_ip,
                        "path": request.url.path,
                        "parameter": key,
                        "value": str_value
                    }
                )
                return JSONResponse(
                    status_code=400,
                    content={"detail": f"Invalid characters detected in path parameter: {key}"}
                )
            
            # Additional validation for file paths
            if "path" in key.lower() or "file" in key.lower() or "dir" in key.lower():
                if not self._is_safe_path(str_value):
                    logger.warning(
                        f"Unsafe path detected in path parameter",
                        {
                            "client_ip": client_ip,
                            "path": request.url.path,
                            "parameter": key,
                            "value": str_value
                        }
                    )
                    return JSONResponse(
                        status_code=400,
                        content={"detail": f"Invalid path in parameter: {key}"}
                    )
        
        # Process the request
        response = await call_next(request)
        
        return response
    
    def _is_safe_path(self, path: str) -> bool:
        """
        Check if a path is safe.
        
        Args:
            path: The path to check
            
        Returns:
            True if the path is safe, False otherwise
        """
        # Normalize path
        normalized_path = os.path.normpath(path)
        
        # Check for path traversal
        if is_path_traversal(normalized_path):
            return False
        
        # Check if the path is within the base path
        if self.base_path:
            full_path = os.path.abspath(os.path.join(self.base_path, normalized_path))
            if not full_path.startswith(os.path.abspath(self.base_path)):
                return False
        
        return True
