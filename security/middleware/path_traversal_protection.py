"""
Path traversal protection middleware for FastAPI.

This middleware adds path traversal protection to HTTP requests.
"""

import os
import re
from typing import Callable, Dict, List, Optional, Set

from fastapi import FastAPI, Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

from security.utils.input_sanitizer import is_path_traversal

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
        Process the request and add path traversal protection.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response
        """
        # Skip path traversal protection for excluded paths
        path = request.url.path
        if any(path.startswith(excluded) for excluded in self.exclude_paths):
            return await call_next(request)
        
        # Check query parameters for path traversal
        for key, value in request.query_params.items():
            if "path" in key.lower() and is_path_traversal(value):
                return JSONResponse(
                    status_code=400,
                    content={"detail": f"Path traversal detected in query parameter: {key}"}
                )
        
        # Check path parameters for path traversal
        for key, value in request.path_params.items():
            if "path" in key.lower() and is_path_traversal(str(value)):
                return JSONResponse(
                    status_code=400,
                    content={"detail": f"Path traversal detected in path parameter: {key}"}
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
