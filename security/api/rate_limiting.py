"""
Rate limiting utilities for Bulo.Cloud Sentinel.

This module provides functions for implementing rate limiting
to protect API endpoints from abuse.
"""

import time
import hashlib
from typing import Dict, List, Optional, Tuple, Union, Callable
from datetime import datetime, timedelta, timezone

from fastapi import FastAPI, Request, Response, status, Depends
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware

from ..logging.secure_logging import get_secure_logger


class RateLimiter:
    """
    Rate limiter for API endpoints.
    """
    
    def __init__(
        self,
        limit: int = 100,
        window: int = 60,
        key_func: Optional[Callable[[Request], str]] = None,
        storage: Optional[Dict[str, List[float]]] = None
    ):
        """
        Initialize the rate limiter.
        
        Args:
            limit: Maximum number of requests per window
            window: Time window in seconds
            key_func: Function to extract key from request
            storage: Storage for request timestamps
        """
        self.limit = limit
        self.window = window
        self.key_func = key_func or self._default_key_func
        self.storage = storage or {}
        self.logger = get_secure_logger("rate_limiter")
    
    def _default_key_func(self, request: Request) -> str:
        """
        Default function to extract key from request.
        
        Args:
            request: Request object
            
        Returns:
            Key for rate limiting
        """
        # Use client IP as key
        client_ip = request.client.host if request.client else "unknown"
        
        # Add path to make it more specific
        path = request.url.path
        
        # Hash the key to avoid storing sensitive information
        key = hashlib.md5(f"{client_ip}:{path}".encode()).hexdigest()
        
        return key
    
    def is_rate_limited(self, request: Request) -> Tuple[bool, int, int]:
        """
        Check if a request is rate limited.
        
        Args:
            request: Request object
            
        Returns:
            Tuple of (is_limited, remaining, reset)
        """
        key = self.key_func(request)
        now = time.time()
        
        # Get request timestamps for this key
        timestamps = self.storage.get(key, [])
        
        # Remove timestamps outside the window
        window_start = now - self.window
        timestamps = [ts for ts in timestamps if ts > window_start]
        
        # Check if limit is exceeded
        is_limited = len(timestamps) >= self.limit
        
        # Add current timestamp if not limited
        if not is_limited:
            timestamps.append(now)
        
        # Update storage
        self.storage[key] = timestamps
        
        # Calculate remaining requests and reset time
        remaining = max(0, self.limit - len(timestamps))
        reset = int(window_start + self.window)
        
        return is_limited, remaining, reset
    
    def __call__(self, request: Request) -> Tuple[bool, int, int]:
        """
        Check if a request is rate limited.
        
        Args:
            request: Request object
            
        Returns:
            Tuple of (is_limited, remaining, reset)
        """
        return self.is_rate_limited(request)


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Middleware for rate limiting.
    """
    
    def __init__(
        self,
        app: FastAPI,
        limit: int = 100,
        window: int = 60,
        key_func: Optional[Callable[[Request], str]] = None,
        exclude_paths: Optional[List[str]] = None,
        storage: Optional[Dict[str, List[float]]] = None,
        error_message: str = "Rate limit exceeded"
    ):
        """
        Initialize the rate limit middleware.
        
        Args:
            app: FastAPI application
            limit: Maximum number of requests per window
            window: Time window in seconds
            key_func: Function to extract key from request
            exclude_paths: Paths to exclude from rate limiting
            storage: Storage for request timestamps
            error_message: Error message for rate limited requests
        """
        super().__init__(app)
        self.limiter = RateLimiter(limit, window, key_func, storage)
        self.exclude_paths = exclude_paths or []
        self.error_message = error_message
        self.logger = get_secure_logger("rate_limit_middleware")
    
    async def dispatch(self, request: Request, call_next):
        """
        Process a request.
        
        Args:
            request: Request object
            call_next: Function to call next middleware
            
        Returns:
            Response
        """
        # Skip rate limiting for excluded paths
        if any(request.url.path.startswith(path) for path in self.exclude_paths):
            return await call_next(request)
        
        # Check rate limit
        is_limited, remaining, reset = self.limiter(request)
        
        # Set rate limit headers
        headers = {
            "X-RateLimit-Limit": str(self.limiter.limit),
            "X-RateLimit-Remaining": str(remaining),
            "X-RateLimit-Reset": str(reset)
        }
        
        # Return error response if rate limited
        if is_limited:
            self.logger.warning(
                "Rate limit exceeded",
                {
                    "client": request.client.host if request.client else "unknown",
                    "path": request.url.path,
                    "method": request.method
                }
            )
            
            return JSONResponse(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                content={"detail": self.error_message},
                headers=headers
            )
        
        # Process request
        response = await call_next(request)
        
        # Add rate limit headers to response
        for key, value in headers.items():
            response.headers[key] = value
        
        return response


def rate_limit(
    limit: int = 100,
    window: int = 60,
    key_func: Optional[Callable[[Request], str]] = None,
    storage: Optional[Dict[str, List[float]]] = None,
    error_message: str = "Rate limit exceeded"
):
    """
    Dependency for rate limiting individual endpoints.
    
    Args:
        limit: Maximum number of requests per window
        window: Time window in seconds
        key_func: Function to extract key from request
        storage: Storage for request timestamps
        error_message: Error message for rate limited requests
        
    Returns:
        Dependency function
    """
    limiter = RateLimiter(limit, window, key_func, storage)
    logger = get_secure_logger("rate_limit_dependency")
    
    def dependency(request: Request, response: Response):
        # Check rate limit
        is_limited, remaining, reset = limiter(request)
        
        # Set rate limit headers
        response.headers["X-RateLimit-Limit"] = str(limit)
        response.headers["X-RateLimit-Remaining"] = str(remaining)
        response.headers["X-RateLimit-Reset"] = str(reset)
        
        # Raise exception if rate limited
        if is_limited:
            logger.warning(
                "Rate limit exceeded",
                {
                    "client": request.client.host if request.client else "unknown",
                    "path": request.url.path,
                    "method": request.method
                }
            )
            
            from fastapi import HTTPException
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail=error_message,
                headers={
                    "X-RateLimit-Limit": str(limit),
                    "X-RateLimit-Remaining": "0",
                    "X-RateLimit-Reset": str(reset)
                }
            )
    
    return Depends(dependency)
