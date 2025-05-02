"""
Unified security middleware for Bulo.Cloud Sentinel.

This module provides a unified approach to security middleware across all components
of the Bulo.Cloud Sentinel platform, combining security headers, CSRF protection,
rate limiting, and error handling.
"""

import logging
import os
from typing import Dict, List, Optional, Set, Union, Callable
from fastapi import FastAPI, Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from security.logging.secure_logging import get_secure_logger
from security.api.unified_security_headers import (
    SecurityHeadersMiddleware,
    DEFAULT_SECURITY_HEADERS,
    DEFAULT_CSP
)
from security.api.csrf_protection import CSRFMiddleware
from security.api.rate_limiting import RateLimitMiddleware, RateLimiter
from security.error_handling.secure_error_handler import (
    configure_error_handlers,
    configure_custom_exception_handlers
)

# Set up logging
logger = get_secure_logger("unified_security")

def configure_security(
    app: FastAPI,
    # Security headers settings
    security_headers: Optional[Dict[str, str]] = None,
    content_security_policy: Optional[str] = None,
    include_csp: bool = True,
    
    # CSRF protection settings
    enable_csrf: bool = True,
    csrf_secret_key: Optional[str] = None,
    csrf_cookie_name: str = "csrf_token",
    csrf_header_name: str = "X-CSRF-Token",
    csrf_cookie_secure: bool = True,
    csrf_cookie_httponly: bool = True,
    csrf_cookie_samesite: str = "Lax",
    csrf_safe_methods: Optional[List[str]] = None,
    
    # Rate limiting settings
    enable_rate_limiting: bool = True,
    rate_limit: int = 100,
    rate_window: int = 60,
    rate_block_duration: int = 300,
    redis_url: Optional[str] = None,
    
    # Common settings
    exclude_paths: Optional[List[str]] = None,
    trusted_hosts: Optional[List[str]] = None,
    
    # Error handling settings
    configure_errors: bool = True
) -> None:
    """
    Configure security middleware for a FastAPI application.
    
    Args:
        app: FastAPI application
        
        # Security headers settings
        security_headers: Security headers to add
        content_security_policy: Content Security Policy header value
        include_csp: Whether to include the Content Security Policy header
        
        # CSRF protection settings
        enable_csrf: Whether to enable CSRF protection
        csrf_secret_key: Secret key for CSRF token generation
        csrf_cookie_name: Name of the CSRF cookie
        csrf_header_name: Name of the CSRF header
        csrf_cookie_secure: Whether to set the Secure flag on the CSRF cookie
        csrf_cookie_httponly: Whether to set the HttpOnly flag on the CSRF cookie
        csrf_cookie_samesite: SameSite attribute for the CSRF cookie
        csrf_safe_methods: HTTP methods that don't require CSRF protection
        
        # Rate limiting settings
        enable_rate_limiting: Whether to enable rate limiting
        rate_limit: Request limit for rate limiting
        rate_window: Time window in seconds for rate limiting
        rate_block_duration: Blocking duration in seconds for rate limiting
        redis_url: Redis connection URL for rate limiting
        
        # Common settings
        exclude_paths: Paths to exclude from security middleware
        trusted_hosts: Trusted hosts for the application
        
        # Error handling settings
        configure_errors: Whether to configure error handlers
    """
    # Set up default values
    exclude_paths = exclude_paths or []
    security_headers = security_headers or DEFAULT_SECURITY_HEADERS.copy()
    content_security_policy = content_security_policy or DEFAULT_CSP
    csrf_safe_methods = csrf_safe_methods or ["GET", "HEAD", "OPTIONS", "TRACE"]
    csrf_secret_key = csrf_secret_key or os.environ.get("SECRET_KEY", "")
    redis_url = redis_url or os.environ.get("REDIS_URL", "redis://localhost:6379/0")
    
    # Log configuration
    logger.info(
        "Configuring security middleware",
        extra={
            "include_csp": include_csp,
            "enable_csrf": enable_csrf,
            "enable_rate_limiting": enable_rate_limiting,
            "configure_errors": configure_errors,
            "exclude_paths": exclude_paths
        }
    )
    
    # Add security headers middleware
    app.add_middleware(
        SecurityHeadersMiddleware,
        headers=security_headers,
        content_security_policy=content_security_policy,
        include_csp=include_csp,
        exclude_paths=exclude_paths
    )
    
    # Add CSRF protection middleware
    if enable_csrf and csrf_secret_key:
        app.add_middleware(
            CSRFMiddleware,
            secret_key=csrf_secret_key,
            cookie_name=csrf_cookie_name,
            header_name=csrf_header_name,
            cookie_secure=csrf_cookie_secure,
            cookie_httponly=csrf_cookie_httponly,
            cookie_samesite=csrf_cookie_samesite,
            safe_methods=csrf_safe_methods,
            exclude_paths=exclude_paths
        )
    
    # Add rate limiting middleware
    if enable_rate_limiting:
        try:
            # Create rate limiter
            rate_limiter = RateLimiter(
                redis_url=redis_url,
                default_limit=rate_limit,
                default_window=rate_window,
                default_block_duration=rate_block_duration
            )
            
            # Add rate limit middleware
            app.add_middleware(
                RateLimitMiddleware,
                rate_limiter=rate_limiter,
                limit=rate_limit,
                window=rate_window,
                block_duration=rate_block_duration,
                exclude_paths=exclude_paths
            )
            
            # Store rate limiter in app state for use in dependencies
            app.state.rate_limiter = rate_limiter
        except Exception as e:
            logger.error(f"Failed to configure rate limiting: {str(e)}")
    
    # Configure error handlers
    if configure_errors:
        configure_error_handlers(app)
        configure_custom_exception_handlers(app)
    
    # Add trusted hosts middleware if provided
    if trusted_hosts:
        from starlette.middleware.trustedhost import TrustedHostMiddleware
        app.add_middleware(
            TrustedHostMiddleware,
            allowed_hosts=trusted_hosts
        )
    
    logger.info("Security middleware configuration complete")
