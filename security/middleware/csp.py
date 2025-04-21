"""
Content Security Policy middleware for FastAPI.

This module provides middleware for implementing Content Security Policy (CSP).
"""

from typing import Callable, Dict, List, Optional, Set, Union

from fastapi import FastAPI, Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

class ContentSecurityPolicyMiddleware(BaseHTTPMiddleware):
    """
    Middleware for implementing Content Security Policy.
    
    This middleware adds a Content-Security-Policy header to HTTP responses.
    """
    
    def __init__(
        self,
        app: FastAPI,
        policy: Optional[Dict[str, Union[str, List[str]]]] = None,
        report_only: bool = False,
        report_uri: Optional[str] = None
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            policy: CSP policy directives
            report_only: Whether to use report-only mode
            report_uri: URI for reporting CSP violations
        """
        super().__init__(app)
        self.policy = policy or {}
        self.report_only = report_only
        self.report_uri = report_uri
        
    def _build_csp_header(self) -> str:
        """
        Build the CSP header value.
        
        Returns:
            CSP header value
        """
        directives = []
        
        for directive, value in self.policy.items():
            if isinstance(value, list):
                directive_value = " ".join(value)
            else:
                directive_value = value
                
            directives.append(f"{directive} {directive_value}")
            
        if self.report_uri:
            directives.append(f"report-uri {self.report_uri}")
            
        return "; ".join(directives)
    
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and add CSP header to the response.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response with CSP header
        """
        response = await call_next(request)
        
        # Build CSP header
        csp_value = self._build_csp_header()
        
        # Add CSP header to response
        header_name = "Content-Security-Policy-Report-Only" if self.report_only else "Content-Security-Policy"
        response.headers[header_name] = csp_value
        
        return response

def get_default_csp_policy() -> Dict[str, Union[str, List[str]]]:
    """
    Get a default CSP policy.
    
    Returns:
        Default CSP policy
    """
    return {
        "default-src": ["'self'"],
        "script-src": ["'self'", "'unsafe-inline'", "'unsafe-eval'"],
        "style-src": ["'self'", "'unsafe-inline'"],
        "img-src": ["'self'", "data:", "blob:"],
        "font-src": ["'self'"],
        "connect-src": ["'self'"],
        "media-src": ["'self'"],
        "object-src": ["'none'"],
        "child-src": ["'self'"],
        "frame-src": ["'self'"],
        "worker-src": ["'self'", "blob:"],
        "frame-ancestors": ["'self'"],
        "form-action": ["'self'"],
        "base-uri": ["'self'"],
        "manifest-src": ["'self'"]
    }

def get_strict_csp_policy() -> Dict[str, Union[str, List[str]]]:
    """
    Get a strict CSP policy.
    
    Returns:
        Strict CSP policy
    """
    return {
        "default-src": ["'none'"],
        "script-src": ["'self'"],
        "style-src": ["'self'"],
        "img-src": ["'self'"],
        "font-src": ["'self'"],
        "connect-src": ["'self'"],
        "media-src": ["'self'"],
        "object-src": ["'none'"],
        "child-src": ["'none'"],
        "frame-src": ["'none'"],
        "worker-src": ["'self'"],
        "frame-ancestors": ["'none'"],
        "form-action": ["'self'"],
        "base-uri": ["'self'"],
        "manifest-src": ["'self'"],
        "upgrade-insecure-requests": ""
    }
