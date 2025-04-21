"""
Security middleware for FastAPI applications.

This module provides middleware for enhancing the security of FastAPI applications.
"""

from fastapi import FastAPI
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response
from typing import Callable, Dict, List, Optional, Union

class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """
    Middleware for adding security headers to responses.
    
    This middleware adds various security headers to HTTP responses to enhance
    the security of the application.
    """
    
    def __init__(
        self,
        app: FastAPI,
        content_security_policy: Optional[str] = None,
        x_frame_options: str = "DENY",
        x_content_type_options: str = "nosniff",
        x_xss_protection: str = "1; mode=block",
        strict_transport_security: str = "max-age=31536000; includeSubDomains",
        referrer_policy: str = "no-referrer",
        permissions_policy: Optional[str] = None,
        cache_control: str = "no-store, no-cache, must-revalidate, max-age=0",
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            content_security_policy: Content-Security-Policy header value
            x_frame_options: X-Frame-Options header value
            x_content_type_options: X-Content-Type-Options header value
            x_xss_protection: X-XSS-Protection header value
            strict_transport_security: Strict-Transport-Security header value
            referrer_policy: Referrer-Policy header value
            permissions_policy: Permissions-Policy header value
            cache_control: Cache-Control header value
        """
        super().__init__(app)
        self.content_security_policy = content_security_policy
        self.x_frame_options = x_frame_options
        self.x_content_type_options = x_content_type_options
        self.x_xss_protection = x_xss_protection
        self.strict_transport_security = strict_transport_security
        self.referrer_policy = referrer_policy
        self.permissions_policy = permissions_policy
        self.cache_control = cache_control
        
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and add security headers to the response.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response with security headers
        """
        response = await call_next(request)
        
        # Add security headers
        response.headers["X-Frame-Options"] = self.x_frame_options
        response.headers["X-Content-Type-Options"] = self.x_content_type_options
        response.headers["X-XSS-Protection"] = self.x_xss_protection
        response.headers["Strict-Transport-Security"] = self.strict_transport_security
        response.headers["Referrer-Policy"] = self.referrer_policy
        response.headers["Cache-Control"] = self.cache_control
        response.headers["Pragma"] = "no-cache"
        
        # Add optional headers
        if self.content_security_policy:
            response.headers["Content-Security-Policy"] = self.content_security_policy
            
        if self.permissions_policy:
            response.headers["Permissions-Policy"] = self.permissions_policy
            
        return response

class RateLimitingMiddleware(BaseHTTPMiddleware):
    """
    Middleware for rate limiting requests.
    
    This middleware implements a simple rate limiting mechanism to prevent
    abuse of the API.
    """
    
    def __init__(
        self,
        app: FastAPI,
        rate_limit: int = 100,  # requests per minute
        rate_limit_window: int = 60,  # seconds
        exclude_paths: Optional[List[str]] = None,
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            rate_limit: Maximum number of requests per window
            rate_limit_window: Time window in seconds
            exclude_paths: List of paths to exclude from rate limiting
        """
        super().__init__(app)
        self.rate_limit = rate_limit
        self.rate_limit_window = rate_limit_window
        self.exclude_paths = exclude_paths or []
        self.request_counts: Dict[str, Dict[str, Union[int, float]]] = {}
        
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and apply rate limiting.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response
        """
        import time
        
        # Skip rate limiting for excluded paths
        path = request.url.path
        if any(path.startswith(excluded) for excluded in self.exclude_paths):
            return await call_next(request)
            
        # Get client IP
        client_ip = request.client.host if request.client else "unknown"
        
        # Check rate limit
        current_time = time.time()
        if client_ip in self.request_counts:
            # Clean up old entries
            if current_time - self.request_counts[client_ip]["timestamp"] > self.rate_limit_window:
                self.request_counts[client_ip] = {"count": 1, "timestamp": current_time}
            else:
                # Increment count
                self.request_counts[client_ip]["count"] += 1
                
                # Check if rate limit exceeded
                if self.request_counts[client_ip]["count"] > self.rate_limit:
                    from fastapi import status
                    from fastapi.responses import JSONResponse
                    
                    # Calculate retry-after time
                    retry_after = int(self.rate_limit_window - (current_time - self.request_counts[client_ip]["timestamp"]))
                    
                    # Return rate limit exceeded response
                    return JSONResponse(
                        status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                        content={"detail": "Rate limit exceeded"},
                        headers={"Retry-After": str(retry_after)}
                    )
        else:
            # First request from this IP
            self.request_counts[client_ip] = {"count": 1, "timestamp": current_time}
            
        # Process the request
        return await call_next(request)
