"""
Unified security headers module for Bulo.Cloud Sentinel.

This module provides a standardized approach to security headers across all components
of the Bulo.Cloud Sentinel platform, ensuring consistent protection against common
web vulnerabilities.
"""

import logging
from typing import Dict, List, Optional, Set, Union
from fastapi import FastAPI
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response
from starlette.types import ASGIApp

from security.logging.secure_logging import get_secure_logger

# Set up logging
logger = get_secure_logger("security_headers")

# Default security headers
DEFAULT_SECURITY_HEADERS = {
    "X-Content-Type-Options": "nosniff",
    "X-Frame-Options": "DENY",
    "X-XSS-Protection": "1; mode=block",
    "Referrer-Policy": "strict-origin-when-cross-origin",
    "Permissions-Policy": "camera=(), microphone=(), geolocation=(), interest-cohort=(), payment=(), usb=(), accelerometer=(), gyroscope=(), magnetometer=()",
    "Cache-Control": "no-store, max-age=0",
    "Pragma": "no-cache",
    "Strict-Transport-Security": "max-age=31536000; includeSubDomains; preload",
    "Cross-Origin-Opener-Policy": "same-origin",
    "Cross-Origin-Embedder-Policy": "require-corp",
    "Cross-Origin-Resource-Policy": "same-origin"
}

# Default Content Security Policy
DEFAULT_CSP = (
    "default-src 'self'; "
    "script-src 'self' 'unsafe-inline'; "  # Allow inline scripts for compatibility
    "style-src 'self' 'unsafe-inline'; "   # Allow inline styles for compatibility
    "img-src 'self' data: https:; "        # Allow data: URLs for images and https images
    "font-src 'self' data:; "              # Allow data: URLs for fonts
    "connect-src 'self' https:; "          # Allow HTTPS connections
    "media-src 'self'; "
    "object-src 'none'; "                  # Block <object>, <embed>, and <applet>
    "child-src 'self'; "
    "frame-src 'self'; "
    "worker-src 'self' blob:; "            # Allow blob: for web workers
    "frame-ancestors 'none'; "             # Prevent framing (similar to X-Frame-Options)
    "form-action 'self'; "                 # Restrict form submissions
    "base-uri 'self'; "                    # Restrict <base> URIs
    "manifest-src 'self'; "
    "upgrade-insecure-requests; "          # Upgrade HTTP to HTTPS
    "block-all-mixed-content"              # Block mixed content
)

class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """
    Middleware for adding security headers to responses.
    """

    def __init__(
        self,
        app: ASGIApp,
        headers: Optional[Dict[str, str]] = None,
        content_security_policy: Optional[str] = None,
        include_csp: bool = True,
        exclude_paths: Optional[List[str]] = None
    ):
        """
        Initialize the security headers middleware.

        Args:
            app: ASGI application
            headers: Security headers to add
            content_security_policy: Content Security Policy header value
            include_csp: Whether to include the Content Security Policy header
            exclude_paths: Paths to exclude from adding headers
        """
        super().__init__(app)
        self.headers = headers or DEFAULT_SECURITY_HEADERS.copy()
        self.content_security_policy = content_security_policy or DEFAULT_CSP
        self.include_csp = include_csp
        self.exclude_paths = exclude_paths or []

        # Log configuration
        logger.info(
            "Configuring security headers middleware",
            extra={
                "headers": self.headers,
                "include_csp": self.include_csp,
                "exclude_paths": self.exclude_paths
            }
        )

    async def dispatch(self, request: Request, call_next) -> Response:
        """
        Dispatch the request and add security headers to the response.

        Args:
            request: Request object
            call_next: Next middleware or route handler

        Returns:
            Response with security headers
        """
        # Process the request
        response = await call_next(request)

        # Check if the path should be excluded
        if any(request.url.path.startswith(path) for path in self.exclude_paths):
            return response

        # Add security headers
        for header_name, header_value in self.headers.items():
            response.headers[header_name] = header_value

        # Add Content Security Policy header if enabled
        if self.include_csp:
            response.headers["Content-Security-Policy"] = self.content_security_policy

        return response

def get_csp_header(
    default_src: Optional[List[str]] = None,
    script_src: Optional[List[str]] = None,
    style_src: Optional[List[str]] = None,
    img_src: Optional[List[str]] = None,
    font_src: Optional[List[str]] = None,
    connect_src: Optional[List[str]] = None,
    media_src: Optional[List[str]] = None,
    object_src: Optional[List[str]] = None,
    child_src: Optional[List[str]] = None,
    frame_src: Optional[List[str]] = None,
    worker_src: Optional[List[str]] = None,
    frame_ancestors: Optional[List[str]] = None,
    form_action: Optional[List[str]] = None,
    base_uri: Optional[List[str]] = None,
    manifest_src: Optional[List[str]] = None,
    report_uri: Optional[str] = None,
    report_to: Optional[str] = None
) -> str:
    """
    Generate a Content Security Policy header value.

    Args:
        default_src: Default source directives
        script_src: Script source directives
        style_src: Style source directives
        img_src: Image source directives
        font_src: Font source directives
        connect_src: Connect source directives
        media_src: Media source directives
        object_src: Object source directives
        child_src: Child source directives
        frame_src: Frame source directives
        worker_src: Worker source directives
        frame_ancestors: Frame ancestors directives
        form_action: Form action directives
        base_uri: Base URI directives
        manifest_src: Manifest source directives
        report_uri: Report URI directive
        report_to: Report-To directive

    Returns:
        Content Security Policy header value
    """
    # Helper function to format directives
    def format_directive(name: str, values: List[str]) -> str:
        if not values:
            return ""
        return f"{name} {' '.join(values)}; "

    # Build the CSP header
    csp = ""

    if default_src:
        csp += format_directive("default-src", default_src)
    else:
        csp += "default-src 'self'; "

    if script_src:
        csp += format_directive("script-src", script_src)

    if style_src:
        csp += format_directive("style-src", style_src)

    if img_src:
        csp += format_directive("img-src", img_src)

    if font_src:
        csp += format_directive("font-src", font_src)

    if connect_src:
        csp += format_directive("connect-src", connect_src)

    if media_src:
        csp += format_directive("media-src", media_src)

    if object_src:
        csp += format_directive("object-src", object_src)
    else:
        csp += "object-src 'none'; "

    if child_src:
        csp += format_directive("child-src", child_src)

    if frame_src:
        csp += format_directive("frame-src", frame_src)

    if worker_src:
        csp += format_directive("worker-src", worker_src)

    if frame_ancestors:
        csp += format_directive("frame-ancestors", frame_ancestors)
    else:
        csp += "frame-ancestors 'none'; "

    if form_action:
        csp += format_directive("form-action", form_action)

    if base_uri:
        csp += format_directive("base-uri", base_uri)

    if manifest_src:
        csp += format_directive("manifest-src", manifest_src)

    if report_uri:
        csp += f"report-uri {report_uri}; "

    if report_to:
        csp += f"report-to {report_to}; "

    return csp.strip()

def get_permissions_policy_header(
    camera: bool = False,
    microphone: bool = False,
    geolocation: bool = False,
    interest_cohort: bool = False,
    accelerometer: bool = False,
    gyroscope: bool = False,
    magnetometer: bool = False,
    payment: bool = False,
    usb: bool = False
) -> str:
    """
    Generate a Permissions-Policy header value.

    Args:
        camera: Whether to allow camera access
        microphone: Whether to allow microphone access
        geolocation: Whether to allow geolocation access
        interest_cohort: Whether to allow interest cohort calculation
        accelerometer: Whether to allow accelerometer access
        gyroscope: Whether to allow gyroscope access
        magnetometer: Whether to allow magnetometer access
        payment: Whether to allow payment API access
        usb: Whether to allow USB API access

    Returns:
        Permissions-Policy header value
    """
    # Helper function to format directives
    def format_directive(name: str, allowed: bool) -> str:
        return f"{name}={'' if allowed else '()'}"

    # Build the Permissions-Policy header
    directives = [
        format_directive("camera", camera),
        format_directive("microphone", microphone),
        format_directive("geolocation", geolocation),
        format_directive("interest-cohort", interest_cohort),
        format_directive("accelerometer", accelerometer),
        format_directive("gyroscope", gyroscope),
        format_directive("magnetometer", magnetometer),
        format_directive("payment", payment),
        format_directive("usb", usb)
    ]

    return ", ".join(directives)

def add_security_headers(
    app: FastAPI,
    headers: Optional[Dict[str, str]] = None,
    content_security_policy: Optional[str] = None,
    include_csp: bool = True,
    exclude_paths: Optional[List[str]] = None
) -> None:
    """
    Add security headers middleware to a FastAPI application.

    Args:
        app: FastAPI application
        headers: Security headers to add
        content_security_policy: Content Security Policy header value
        include_csp: Whether to include the Content Security Policy header
        exclude_paths: Paths to exclude from adding headers
    """
    # Add security headers middleware
    app.add_middleware(
        SecurityHeadersMiddleware,
        headers=headers,
        content_security_policy=content_security_policy,
        include_csp=include_csp,
        exclude_paths=exclude_paths
    )

    logger.info("Added security headers middleware to FastAPI application")
