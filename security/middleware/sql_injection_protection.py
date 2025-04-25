"""
SQL injection protection middleware for FastAPI.

This middleware adds SQL injection protection to HTTP requests.
"""

import json
import re
from typing import Callable, Dict, List, Optional, Set, Any

from fastapi import FastAPI, Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

from security.utils.input_sanitizer import is_sql_injection

class SQLInjectionProtectionMiddleware(BaseHTTPMiddleware):
    """
    Middleware for SQL injection protection.
    
    This middleware adds SQL injection protection to HTTP requests.
    """
    
    def __init__(
        self,
        app: FastAPI,
        exclude_paths: Optional[List[str]] = None,
        exclude_content_types: Optional[List[str]] = None,
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            exclude_paths: List of paths to exclude from SQL injection protection
            exclude_content_types: List of content types to exclude from SQL injection protection
        """
        super().__init__(app)
        self.exclude_paths = exclude_paths or []
        self.exclude_content_types = exclude_content_types or []
    
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and add SQL injection protection.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response
        """
        # Skip SQL injection protection for excluded paths
        path = request.url.path
        if any(path.startswith(excluded) for excluded in self.exclude_paths):
            return await call_next(request)
        
        # Skip SQL injection protection for excluded content types
        content_type = request.headers.get("content-type", "")
        if any(ct in content_type for ct in self.exclude_content_types):
            return await call_next(request)
        
        # Check query parameters for SQL injection
        for key, value in request.query_params.items():
            if is_sql_injection(value):
                return JSONResponse(
                    status_code=400,
                    content={"detail": f"SQL injection detected in query parameter: {key}"}
                )
        
        # Check path parameters for SQL injection
        for key, value in request.path_params.items():
            if is_sql_injection(str(value)):
                return JSONResponse(
                    status_code=400,
                    content={"detail": f"SQL injection detected in path parameter: {key}"}
                )
        
        # Check form data for SQL injection
        if "application/x-www-form-urlencoded" in content_type:
            form_data = await request.form()
            for key, value in form_data.items():
                if is_sql_injection(str(value)):
                    return JSONResponse(
                        status_code=400,
                        content={"detail": f"SQL injection detected in form data: {key}"}
                    )
        
        # Check JSON body for SQL injection
        if "application/json" in content_type:
            try:
                body = await request.json()
                if await self._check_json_for_sql_injection(body):
                    return JSONResponse(
                        status_code=400,
                        content={"detail": "SQL injection detected in JSON body"}
                    )
            except json.JSONDecodeError:
                # Invalid JSON, skip SQL injection check
                pass
        
        # Process the request
        response = await call_next(request)
        
        return response
    
    async def _check_json_for_sql_injection(self, data: Any, path: str = "") -> bool:
        """
        Check JSON data for SQL injection.
        
        Args:
            data: The JSON data
            path: The current path in the JSON data
            
        Returns:
            True if SQL injection is detected, False otherwise
        """
        if isinstance(data, dict):
            for key, value in data.items():
                current_path = f"{path}.{key}" if path else key
                if isinstance(value, (dict, list)):
                    if await self._check_json_for_sql_injection(value, current_path):
                        return True
                elif isinstance(value, str) and is_sql_injection(value):
                    return True
        elif isinstance(data, list):
            for i, item in enumerate(data):
                current_path = f"{path}[{i}]"
                if isinstance(item, (dict, list)):
                    if await self._check_json_for_sql_injection(item, current_path):
                        return True
                elif isinstance(item, str) and is_sql_injection(item):
                    return True
        
        return False
