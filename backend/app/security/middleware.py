"""
Security middleware for the Bulo.Cloud Sentinel backend.

This module provides middleware for enhancing security, including:
- Secure headers
- CSRF protection
- Rate limiting
"""

from fastapi import FastAPI, Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.middleware.cors import CORSMiddleware
from starlette.middleware.trustedhost import TrustedHostMiddleware
from starlette.middleware.gzip import GZipMiddleware
from slowapi import Limiter
from slowapi.middleware import SlowAPIMiddleware
from slowapi.errors import RateLimitExceeded
from slowapi.util import get_remote_address
import time
import secrets
import logging
from typing import Callable, List, Dict, Any, Optional

logger = logging.getLogger(__name__)


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """
    Middleware for adding security headers to responses.
    
    This middleware adds various security headers to responses to enhance security.
    """
    
    def __init__(
        self,
        app,
        content_security_policy: Optional[str] = None,
        hsts_max_age: int = 31536000,  # 1 year
        include_subdomains: bool = True,
        preload: bool = False,
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application.
            content_security_policy: Content Security Policy header value.
            hsts_max_age: HSTS max age in seconds.
            include_subdomains: Whether to include subdomains in HSTS.
            preload: Whether to preload HSTS.
        """
        super().__init__(app)
        self.content_security_policy = content_security_policy
        self.hsts_max_age = hsts_max_age
        self.include_subdomains = include_subdomains
        self.preload = preload
    
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Dispatch the request and add security headers to the response.
        
        Args:
            request: The request.
            call_next: The next middleware or route handler.
            
        Returns:
            Response: The response with security headers.
        """
        response = await call_next(request)
        
        # Add security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
        response.headers["Permissions-Policy"] = "camera=(), microphone=(), geolocation=(), interest-cohort=()"
        
        # Add Content-Security-Policy header if provided
        if self.content_security_policy:
            response.headers["Content-Security-Policy"] = self.content_security_policy
        
        # Add Strict-Transport-Security header
        hsts_header = f"max-age={self.hsts_max_age}"
        if self.include_subdomains:
            hsts_header += "; includeSubDomains"
        if self.preload:
            hsts_header += "; preload"
        response.headers["Strict-Transport-Security"] = hsts_header
        
        return response


class CSRFMiddleware(BaseHTTPMiddleware):
    """
    Middleware for CSRF protection.
    
    This middleware implements CSRF protection using the Double Submit Cookie pattern.
    """
    
    def __init__(
        self,
        app,
        cookie_name: str = "csrf_token",
        header_name: str = "X-CSRF-Token",
        cookie_secure: bool = True,
        cookie_httponly: bool = True,
        cookie_samesite: str = "Lax",
        exempt_paths: List[str] = None,
        exempt_methods: List[str] = None,
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application.
            cookie_name: The name of the CSRF cookie.
            header_name: The name of the CSRF header.
            cookie_secure: Whether the cookie should be secure.
            cookie_httponly: Whether the cookie should be HTTP-only.
            cookie_samesite: The SameSite attribute of the cookie.
            exempt_paths: Paths exempt from CSRF protection.
            exempt_methods: HTTP methods exempt from CSRF protection.
        """
        super().__init__(app)
        self.cookie_name = cookie_name
        self.header_name = header_name
        self.cookie_secure = cookie_secure
        self.cookie_httponly = cookie_httponly
        self.cookie_samesite = cookie_samesite
        self.exempt_paths = exempt_paths or ["/docs", "/redoc", "/openapi.json"]
        self.exempt_methods = exempt_methods or ["GET", "HEAD", "OPTIONS"]
    
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Dispatch the request and implement CSRF protection.
        
        Args:
            request: The request.
            call_next: The next middleware or route handler.
            
        Returns:
            Response: The response.
        """
        # Check if the request is exempt from CSRF protection
        if request.method in self.exempt_methods:
            response = await call_next(request)
            return response
        
        for path in self.exempt_paths:
            if request.url.path.startswith(path):
                response = await call_next(request)
                return response
        
        # Get the CSRF token from the cookie
        csrf_cookie = request.cookies.get(self.cookie_name)
        
        # Get the CSRF token from the header
        csrf_header = request.headers.get(self.header_name)
        
        # If this is the first request, generate a CSRF token
        if not csrf_cookie:
            response = await call_next(request)
            csrf_token = secrets.token_hex(32)
            response.set_cookie(
                self.cookie_name,
                csrf_token,
                secure=self.cookie_secure,
                httponly=self.cookie_httponly,
                samesite=self.cookie_samesite,
            )
            return response
        
        # Verify the CSRF token
        if not csrf_header or csrf_cookie != csrf_header:
            return Response(
                content="CSRF token missing or invalid",
                status_code=403,
                media_type="text/plain",
            )
        
        # Continue with the request
        response = await call_next(request)
        return response


def setup_security_middleware(
    app: FastAPI,
    cors_origins: List[str] = None,
    trusted_hosts: List[str] = None,
    rate_limit: str = "60/minute",
    content_security_policy: str = None,
    enable_csrf: bool = True,
    enable_gzip: bool = True,
) -> None:
    """
    Set up security middleware for the FastAPI application.
    
    Args:
        app: The FastAPI application.
        cors_origins: Allowed CORS origins.
        trusted_hosts: Trusted hosts.
        rate_limit: Rate limit string (e.g., "60/minute").
        content_security_policy: Content Security Policy header value.
        enable_csrf: Whether to enable CSRF protection.
        enable_gzip: Whether to enable GZip compression.
    """
    # Set up CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=cors_origins or ["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    
    # Set up trusted host middleware
    if trusted_hosts:
        app.add_middleware(
            TrustedHostMiddleware,
            allowed_hosts=trusted_hosts,
        )
    
    # Set up GZip middleware
    if enable_gzip:
        app.add_middleware(GZipMiddleware, minimum_size=1000)
    
    # Set up security headers middleware
    app.add_middleware(
        SecurityHeadersMiddleware,
        content_security_policy=content_security_policy,
    )
    
    # Set up CSRF middleware
    if enable_csrf:
        app.add_middleware(
            CSRFMiddleware,
            cookie_secure=True,
            cookie_httponly=True,
            cookie_samesite="Lax",
        )
    
    # Set up rate limiting middleware
    limiter = Limiter(key_func=get_remote_address, default_limits=[rate_limit])
    app.state.limiter = limiter
    app.add_middleware(SlowAPIMiddleware)
    
    @app.exception_handler(RateLimitExceeded)
    async def rate_limit_handler(request: Request, exc: RateLimitExceeded):
        return Response(
            content="Rate limit exceeded",
            status_code=429,
            media_type="text/plain",
        )
