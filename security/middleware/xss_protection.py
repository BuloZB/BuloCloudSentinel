"""
XSS protection middleware for FastAPI.

This middleware adds XSS protection headers to HTTP responses.
"""

from typing import Callable, Dict, List, Optional, Set

from fastapi import FastAPI, Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

class XSSProtectionMiddleware(BaseHTTPMiddleware):
    """
    Middleware for XSS protection.
    
    This middleware adds XSS protection headers to HTTP responses.
    """
    
    def __init__(
        self,
        app: FastAPI,
        mode: str = "block",
        include_content_type: Optional[List[str]] = None,
        exclude_paths: Optional[List[str]] = None,
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            mode: XSS protection mode (block, sanitize)
            include_content_type: List of content types to include
            exclude_paths: List of paths to exclude
        """
        super().__init__(app)
        self.mode = mode
        self.include_content_type = include_content_type or ["text/html", "application/xhtml+xml"]
        self.exclude_paths = exclude_paths or []
    
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and add XSS protection headers to the response.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response with XSS protection headers
        """
        # Process the request
        response = await call_next(request)
        
        # Skip XSS protection for excluded paths
        path = request.url.path
        if any(path.startswith(excluded) for excluded in self.exclude_paths):
            return response
        
        # Skip XSS protection for non-HTML content
        content_type = response.headers.get("content-type", "")
        if not any(ct in content_type for ct in self.include_content_type):
            return response
        
        # Add XSS protection headers
        response.headers["X-XSS-Protection"] = f"1; mode={self.mode}"
        
        # Add Content-Security-Policy header
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; "
            "script-src 'self' 'unsafe-inline'; "
            "style-src 'self' 'unsafe-inline'; "
            "img-src 'self' data:; "
            "font-src 'self'; "
            "connect-src 'self'; "
            "frame-ancestors 'none'; "
            "form-action 'self'; "
            "base-uri 'self'; "
            "object-src 'none'"
        )
        
        return response
