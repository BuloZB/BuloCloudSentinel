"""
Security middleware for FastAPI applications.

This module provides middleware for enhancing the security of FastAPI applications.
"""

import time
import logging
from datetime import datetime, timedelta
from fastapi import FastAPI, status
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response
from typing import Callable, Dict, List, Optional, Union, Set

# Set up logging
log = logging.getLogger(__name__)

# Default Content Security Policy
DEFAULT_CSP = (
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

# Default Permissions Policy
DEFAULT_PERMISSIONS_POLICY = (
    "accelerometer=(), "
    "camera=(), "
    "geolocation=(), "
    "gyroscope=(), "
    "magnetometer=(), "
    "microphone=(), "
    "payment=(), "
    "usb=()"
)

class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """
    Middleware for adding security headers to responses.

    This middleware adds various security headers to HTTP responses to enhance
    the security of the application, including Content Security Policy (CSP),
    Strict Transport Security (HSTS), and other security headers.
    """

    def __init__(
        self,
        app: FastAPI,
        content_security_policy: Optional[str] = DEFAULT_CSP,
        x_frame_options: str = "DENY",
        x_content_type_options: str = "nosniff",
        x_xss_protection: str = "1; mode=block",
        strict_transport_security: str = "max-age=31536000; includeSubDomains; preload",
        referrer_policy: str = "strict-origin-when-cross-origin",
        permissions_policy: Optional[str] = DEFAULT_PERMISSIONS_POLICY,
        cache_control: str = "no-store, no-cache, must-revalidate, max-age=0",
        report_to: Optional[str] = None,
        report_uri: Optional[str] = None,
        cross_origin_opener_policy: str = "same-origin",
        cross_origin_embedder_policy: str = "require-corp",
        cross_origin_resource_policy: str = "same-origin",
    ):
        """
        Initialize the middleware with comprehensive security headers.

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
            report_to: Report-To header value for CSP violation reporting
            report_uri: Report-URI header value for CSP violation reporting
            cross_origin_opener_policy: Cross-Origin-Opener-Policy header value
            cross_origin_embedder_policy: Cross-Origin-Embedder-Policy header value
            cross_origin_resource_policy: Cross-Origin-Resource-Policy header value
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
        self.report_to = report_to
        self.report_uri = report_uri
        self.cross_origin_opener_policy = cross_origin_opener_policy
        self.cross_origin_embedder_policy = cross_origin_embedder_policy
        self.cross_origin_resource_policy = cross_origin_resource_policy

        log.info("SecurityHeadersMiddleware initialized with CSP and security headers")

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and add security headers to the response.

        Args:
            request: The HTTP request
            call_next: The next middleware or route handler

        Returns:
            The HTTP response with security headers
        """
        try:
            response = await call_next(request)

            # Add security headers
            response.headers["X-Frame-Options"] = self.x_frame_options
            response.headers["X-Content-Type-Options"] = self.x_content_type_options
            response.headers["X-XSS-Protection"] = self.x_xss_protection
            response.headers["Strict-Transport-Security"] = self.strict_transport_security
            response.headers["Referrer-Policy"] = self.referrer_policy
            response.headers["Cache-Control"] = self.cache_control
            response.headers["Pragma"] = "no-cache"

            # Add Cross-Origin headers
            response.headers["Cross-Origin-Opener-Policy"] = self.cross_origin_opener_policy
            response.headers["Cross-Origin-Embedder-Policy"] = self.cross_origin_embedder_policy
            response.headers["Cross-Origin-Resource-Policy"] = self.cross_origin_resource_policy

            # Add optional headers
            if self.content_security_policy:
                response.headers["Content-Security-Policy"] = self.content_security_policy

            if self.permissions_policy:
                response.headers["Permissions-Policy"] = self.permissions_policy

            if self.report_to:
                response.headers["Report-To"] = self.report_to

            if self.report_uri:
                response.headers["Report-URI"] = self.report_uri

            # Add security headers for API responses
            if request.url.path.startswith("/api/"):
                # Additional API-specific headers
                response.headers["Access-Control-Allow-Credentials"] = "true"
                response.headers["Access-Control-Allow-Methods"] = "GET, POST, PUT, DELETE, OPTIONS"
                response.headers["Access-Control-Allow-Headers"] = "Authorization, Content-Type, Accept"
                response.headers["Access-Control-Max-Age"] = "3600"

            return response
        except Exception as e:
            log.error(f"Error in SecurityHeadersMiddleware: {str(e)}")
            raise

class RateLimitingMiddleware(BaseHTTPMiddleware):
    """
    Middleware for rate limiting requests.

    This middleware implements an advanced rate limiting mechanism to prevent
    abuse of the API, with different limits for different endpoints and
    authentication states.
    """

    def __init__(
        self,
        app: FastAPI,
        default_rate_limit: int = 100,  # requests per minute
        auth_rate_limit: int = 200,  # requests per minute for authenticated users
        sensitive_rate_limit: int = 10,  # requests per minute for sensitive endpoints
        rate_limit_window: int = 60,  # seconds
        exclude_paths: Optional[List[str]] = None,
        sensitive_paths: Optional[List[str]] = None,
        auth_header_name: str = "Authorization",
        max_request_counts: int = 10000,  # Maximum number of IPs to track
    ):
        """
        Initialize the middleware with advanced rate limiting options.

        Args:
            app: The FastAPI application
            default_rate_limit: Maximum number of requests per window for unauthenticated users
            auth_rate_limit: Maximum number of requests per window for authenticated users
            sensitive_rate_limit: Maximum number of requests per window for sensitive endpoints
            rate_limit_window: Time window in seconds
            exclude_paths: List of paths to exclude from rate limiting
            sensitive_paths: List of paths that are considered sensitive (login, register, etc.)
            auth_header_name: Name of the authentication header
            max_request_counts: Maximum number of IPs to track to prevent memory issues
        """
        super().__init__(app)
        self.default_rate_limit = default_rate_limit
        self.auth_rate_limit = auth_rate_limit
        self.sensitive_rate_limit = sensitive_rate_limit
        self.rate_limit_window = rate_limit_window
        self.exclude_paths = exclude_paths or []
        self.sensitive_paths = sensitive_paths or [
            "/api/auth/login",
            "/api/auth/register",
            "/api/auth/reset-password",
            "/api/auth/token",
            "/api/auth/refresh"
        ]
        self.auth_header_name = auth_header_name
        self.max_request_counts = max_request_counts
        self.request_counts: Dict[str, Dict[str, Union[int, float]]] = {}

        log.info(f"RateLimitingMiddleware initialized with default limit: {default_rate_limit}/min, "
                f"auth limit: {auth_rate_limit}/min, sensitive limit: {sensitive_rate_limit}/min")

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and apply rate limiting based on path and authentication.

        Args:
            request: The HTTP request
            call_next: The next middleware or route handler

        Returns:
            The HTTP response
        """
        try:
            # Skip rate limiting for excluded paths
            path = request.url.path
            if any(path.startswith(excluded) for excluded in self.exclude_paths):
                return await call_next(request)

            # Get client IP
            client_ip = request.client.host if request.client else "unknown"

            # Determine if the request is authenticated
            is_authenticated = self.auth_header_name in request.headers

            # Determine if the path is sensitive
            is_sensitive = any(path.startswith(sensitive) for sensitive in self.sensitive_paths)

            # Determine the appropriate rate limit
            if is_sensitive:
                rate_limit = self.sensitive_rate_limit
            elif is_authenticated:
                rate_limit = self.auth_rate_limit
            else:
                rate_limit = self.default_rate_limit

            # Create a key that includes the path type for separate limits
            key = f"{client_ip}:{path}" if is_sensitive else client_ip

            # Check rate limit
            current_time = time.time()

            # Clean up old entries if we're tracking too many IPs
            if len(self.request_counts) > self.max_request_counts:
                # Remove oldest entries
                oldest_keys = sorted(
                    self.request_counts.keys(),
                    key=lambda k: self.request_counts[k]["timestamp"]
                )[:int(self.max_request_counts * 0.2)]  # Remove oldest 20%

                for old_key in oldest_keys:
                    del self.request_counts[old_key]

                log.warning(f"Rate limiting cache cleanup: removed {len(oldest_keys)} oldest entries")

            if key in self.request_counts:
                # Clean up old entries
                if current_time - self.request_counts[key]["timestamp"] > self.rate_limit_window:
                    self.request_counts[key] = {"count": 1, "timestamp": current_time}
                else:
                    # Increment count
                    self.request_counts[key]["count"] += 1

                    # Check if rate limit exceeded
                    if self.request_counts[key]["count"] > rate_limit:
                        # Calculate retry-after time
                        retry_after = int(self.rate_limit_window - (current_time - self.request_counts[key]["timestamp"]))

                        # Log rate limit exceeded
                        log.warning(f"Rate limit exceeded for {key}: {self.request_counts[key]['count']} requests in {self.rate_limit_window}s")

                        # Return rate limit exceeded response
                        return JSONResponse(
                            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                            content={
                                "detail": "Rate limit exceeded",
                                "limit": rate_limit,
                                "window": self.rate_limit_window,
                                "retry_after": retry_after
                            },
                            headers={
                                "Retry-After": str(retry_after),
                                "X-RateLimit-Limit": str(rate_limit),
                                "X-RateLimit-Remaining": "0",
                                "X-RateLimit-Reset": str(int(current_time + retry_after))
                            }
                        )

                    # Add rate limit headers to response
                    response = await call_next(request)
                    remaining = max(0, rate_limit - self.request_counts[key]["count"])
                    response.headers["X-RateLimit-Limit"] = str(rate_limit)
                    response.headers["X-RateLimit-Remaining"] = str(remaining)
                    response.headers["X-RateLimit-Reset"] = str(int(self.request_counts[key]["timestamp"] + self.rate_limit_window))
                    return response
            else:
                # First request from this IP/path
                self.request_counts[key] = {"count": 1, "timestamp": current_time}

            # Process the request
            response = await call_next(request)

            # Add rate limit headers to response
            response.headers["X-RateLimit-Limit"] = str(rate_limit)
            response.headers["X-RateLimit-Remaining"] = str(rate_limit - 1)
            response.headers["X-RateLimit-Reset"] = str(int(current_time + self.rate_limit_window))

            return response
        except Exception as e:
            log.error(f"Error in RateLimitingMiddleware: {str(e)}")
            raise
