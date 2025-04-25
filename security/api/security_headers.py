"""
Security headers utilities for Bulo.Cloud Sentinel.

This module provides functions for adding security headers to API responses
to protect against various attacks.
"""

from typing import Dict, List, Optional, Union

from fastapi import FastAPI, Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from ..logging.secure_logging import get_secure_logger


# Default security headers
DEFAULT_SECURITY_HEADERS = {
    "X-Content-Type-Options": "nosniff",
    "X-Frame-Options": "DENY",
    "X-XSS-Protection": "1; mode=block",
    "Strict-Transport-Security": "max-age=31536000; includeSubDomains",
    "Content-Security-Policy": "default-src 'self'; script-src 'self'; object-src 'none'; frame-ancestors 'none'",
    "Referrer-Policy": "strict-origin-when-cross-origin",
    "Cache-Control": "no-store, max-age=0",
    "Permissions-Policy": "accelerometer=(), camera=(), geolocation=(), gyroscope=(), magnetometer=(), microphone=(), payment=(), usb=()"
}


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """
    Middleware for adding security headers to responses.
    """
    
    def __init__(
        self,
        app: ASGIApp,
        headers: Optional[Dict[str, str]] = None,
        exclude_paths: Optional[List[str]] = None
    ):
        """
        Initialize the security headers middleware.
        
        Args:
            app: ASGI application
            headers: Security headers to add
            exclude_paths: Paths to exclude from adding headers
        """
        super().__init__(app)
        self.headers = headers or DEFAULT_SECURITY_HEADERS
        self.exclude_paths = exclude_paths or []
        self.logger = get_secure_logger("security_headers")
        
        # Log configuration
        self.logger.info(
            "Configuring security headers",
            {
                "headers": self.headers,
                "exclude_paths": self.exclude_paths
            }
        )
    
    async def dispatch(self, request: Request, call_next):
        """
        Process a request.
        
        Args:
            request: Request object
            call_next: Function to call next middleware
            
        Returns:
            Response
        """
        # Process request
        response = await call_next(request)
        
        # Skip excluded paths
        if any(request.url.path.startswith(path) for path in self.exclude_paths):
            return response
        
        # Add security headers
        for name, value in self.headers.items():
            response.headers[name] = value
        
        return response


def add_security_headers(
    app: FastAPI,
    headers: Optional[Dict[str, str]] = None,
    exclude_paths: Optional[List[str]] = None
):
    """
    Add security headers middleware to a FastAPI application.
    
    Args:
        app: FastAPI application
        headers: Security headers to add
        exclude_paths: Paths to exclude from adding headers
    """
    app.add_middleware(
        SecurityHeadersMiddleware,
        headers=headers,
        exclude_paths=exclude_paths
    )


def get_csp_header(
    default_src: Union[List[str], str] = "'self'",
    script_src: Optional[Union[List[str], str]] = None,
    style_src: Optional[Union[List[str], str]] = None,
    img_src: Optional[Union[List[str], str]] = None,
    connect_src: Optional[Union[List[str], str]] = None,
    font_src: Optional[Union[List[str], str]] = None,
    object_src: Optional[Union[List[str], str]] = None,
    media_src: Optional[Union[List[str], str]] = None,
    frame_src: Optional[Union[List[str], str]] = None,
    frame_ancestors: Optional[Union[List[str], str]] = None,
    report_uri: Optional[str] = None,
    report_to: Optional[str] = None
) -> str:
    """
    Generate a Content-Security-Policy header value.
    
    Args:
        default_src: Default source directive
        script_src: Script source directive
        style_src: Style source directive
        img_src: Image source directive
        connect_src: Connect source directive
        font_src: Font source directive
        object_src: Object source directive
        media_src: Media source directive
        frame_src: Frame source directive
        frame_ancestors: Frame ancestors directive
        report_uri: URI to report violations to
        report_to: Reporting group to send violation reports to
        
    Returns:
        CSP header value
    """
    # Helper function to format directive
    def format_directive(name: str, value: Optional[Union[List[str], str]]) -> Optional[str]:
        if value is None:
            return None
        
        if isinstance(value, list):
            value = " ".join(value)
        
        return f"{name} {value}"
    
    # Build directives
    directives = []
    
    # Add default-src
    directives.append(format_directive("default-src", default_src))
    
    # Add other directives if provided
    if script_src:
        directives.append(format_directive("script-src", script_src))
    
    if style_src:
        directives.append(format_directive("style-src", style_src))
    
    if img_src:
        directives.append(format_directive("img-src", img_src))
    
    if connect_src:
        directives.append(format_directive("connect-src", connect_src))
    
    if font_src:
        directives.append(format_directive("font-src", font_src))
    
    if object_src:
        directives.append(format_directive("object-src", object_src))
    
    if media_src:
        directives.append(format_directive("media-src", media_src))
    
    if frame_src:
        directives.append(format_directive("frame-src", frame_src))
    
    if frame_ancestors:
        directives.append(format_directive("frame-ancestors", frame_ancestors))
    
    # Add reporting directives
    if report_uri:
        directives.append(format_directive("report-uri", report_uri))
    
    if report_to:
        directives.append(format_directive("report-to", report_to))
    
    # Join directives
    return "; ".join(filter(None, directives))


def get_permissions_policy_header(
    accelerometer: Optional[Union[List[str], str]] = "()",
    camera: Optional[Union[List[str], str]] = "()",
    geolocation: Optional[Union[List[str], str]] = "()",
    gyroscope: Optional[Union[List[str], str]] = "()",
    magnetometer: Optional[Union[List[str], str]] = "()",
    microphone: Optional[Union[List[str], str]] = "()",
    payment: Optional[Union[List[str], str]] = "()",
    usb: Optional[Union[List[str], str]] = "()",
    **kwargs
) -> str:
    """
    Generate a Permissions-Policy header value.
    
    Args:
        accelerometer: Accelerometer permission policy
        camera: Camera permission policy
        geolocation: Geolocation permission policy
        gyroscope: Gyroscope permission policy
        magnetometer: Magnetometer permission policy
        microphone: Microphone permission policy
        payment: Payment permission policy
        usb: USB permission policy
        **kwargs: Additional permission policies
        
    Returns:
        Permissions-Policy header value
    """
    # Helper function to format directive
    def format_directive(name: str, value: Optional[Union[List[str], str]]) -> Optional[str]:
        if value is None:
            return None
        
        if isinstance(value, list):
            value = " ".join(value)
        
        return f"{name}={value}"
    
    # Build directives
    directives = []
    
    # Add standard directives
    directives.append(format_directive("accelerometer", accelerometer))
    directives.append(format_directive("camera", camera))
    directives.append(format_directive("geolocation", geolocation))
    directives.append(format_directive("gyroscope", gyroscope))
    directives.append(format_directive("magnetometer", magnetometer))
    directives.append(format_directive("microphone", microphone))
    directives.append(format_directive("payment", payment))
    directives.append(format_directive("usb", usb))
    
    # Add additional directives
    for name, value in kwargs.items():
        directives.append(format_directive(name, value))
    
    # Join directives
    return ", ".join(filter(None, directives))
