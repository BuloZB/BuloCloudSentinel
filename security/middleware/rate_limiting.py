"""
Rate limiting middleware for FastAPI.

This middleware adds rate limiting to HTTP requests.
"""

import time
from typing import Callable, Dict, List, Optional, Set, Tuple, Union

from fastapi import FastAPI, Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

class RateLimitingMiddleware(BaseHTTPMiddleware):
    """
    Middleware for rate limiting.
    
    This middleware adds rate limiting to HTTP requests.
    """
    
    def __init__(
        self,
        app: FastAPI,
        limit: int = 100,
        window: int = 60,
        key_func: Optional[Callable[[Request], str]] = None,
        exclude_paths: Optional[List[str]] = None,
        storage: Optional[Dict[str, Dict[str, Union[int, float]]]] = None,
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            limit: Maximum number of requests per window
            window: Time window in seconds
            key_func: Function to extract the rate limiting key from the request
            exclude_paths: List of paths to exclude from rate limiting
            storage: Storage for rate limiting data (default: in-memory)
        """
        super().__init__(app)
        self.limit = limit
        self.window = window
        self.key_func = key_func or self._default_key_func
        self.exclude_paths = exclude_paths or []
        self.storage = storage or {}
    
    def _default_key_func(self, request: Request) -> str:
        """
        Default function to extract the rate limiting key from the request.
        
        Args:
            request: The HTTP request
            
        Returns:
            The rate limiting key
        """
        # Use client IP as the key
        forwarded = request.headers.get("X-Forwarded-For")
        if forwarded:
            ip = forwarded.split(",")[0].strip()
        else:
            ip = request.client.host if request.client else "unknown"
        
        return f"{ip}:{request.url.path}"
    
    def _is_rate_limited(self, key: str) -> Tuple[bool, int, int]:
        """
        Check if a request is rate limited.
        
        Args:
            key: The rate limiting key
            
        Returns:
            Tuple of (is_limited, remaining, reset)
        """
        current_time = time.time()
        
        # Get or create rate limiting data for the key
        if key not in self.storage:
            self.storage[key] = {
                "count": 0,
                "reset": current_time + self.window
            }
        
        data = self.storage[key]
        
        # Reset count if the window has expired
        if current_time > data["reset"]:
            data["count"] = 0
            data["reset"] = current_time + self.window
        
        # Increment count
        data["count"] += 1
        
        # Check if the limit is exceeded
        is_limited = data["count"] > self.limit
        remaining = max(0, self.limit - data["count"])
        reset = int(data["reset"] - current_time)
        
        return is_limited, remaining, reset
    
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and apply rate limiting.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response
        """
        # Skip rate limiting for excluded paths
        path = request.url.path
        if any(path.startswith(excluded) for excluded in self.exclude_paths):
            return await call_next(request)
        
        # Get the rate limiting key
        key = self.key_func(request)
        
        # Check if the request is rate limited
        is_limited, remaining, reset = self._is_rate_limited(key)
        
        # Add rate limiting headers
        headers = {
            "X-RateLimit-Limit": str(self.limit),
            "X-RateLimit-Remaining": str(remaining),
            "X-RateLimit-Reset": str(reset)
        }
        
        # Return 429 Too Many Requests if the limit is exceeded
        if is_limited:
            return JSONResponse(
                status_code=429,
                content={"detail": "Too many requests"},
                headers=headers
            )
        
        # Process the request
        response = await call_next(request)
        
        # Add rate limiting headers to the response
        for name, value in headers.items():
            response.headers[name] = value
        
        return response
