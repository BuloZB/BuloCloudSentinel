"""
CSRF protection middleware for FastAPI.

This module provides middleware for protecting against Cross-Site Request Forgery (CSRF) attacks.
"""

import secrets
import time
from typing import Callable, Dict, List, Optional, Set

from fastapi import FastAPI, Request, Response, HTTPException, status
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

class CSRFMiddleware(BaseHTTPMiddleware):
    """
    Middleware for CSRF protection.
    
    This middleware implements the Double Submit Cookie pattern for CSRF protection.
    """
    
    def __init__(
        self,
        app: FastAPI,
        secret_key: str,
        cookie_name: str = "csrf_token",
        header_name: str = "X-CSRF-Token",
        cookie_secure: bool = True,
        cookie_httponly: bool = True,
        cookie_samesite: str = "lax",
        cookie_max_age: int = 3600,  # 1 hour
        safe_methods: Optional[Set[str]] = None,
        exclude_paths: Optional[List[str]] = None
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            secret_key: Secret key for token generation
            cookie_name: Name of the CSRF cookie
            header_name: Name of the CSRF header
            cookie_secure: Whether to set the Secure flag on the cookie
            cookie_httponly: Whether to set the HttpOnly flag on the cookie
            cookie_samesite: SameSite attribute for the cookie
            cookie_max_age: Max age of the cookie in seconds
            safe_methods: HTTP methods that don't require CSRF protection
            exclude_paths: Paths to exclude from CSRF protection
        """
        super().__init__(app)
        self.secret_key = secret_key
        self.cookie_name = cookie_name
        self.header_name = header_name
        self.cookie_secure = cookie_secure
        self.cookie_httponly = cookie_httponly
        self.cookie_samesite = cookie_samesite
        self.cookie_max_age = cookie_max_age
        self.safe_methods = safe_methods or {"GET", "HEAD", "OPTIONS"}
        self.exclude_paths = exclude_paths or []
        
    def _generate_csrf_token(self) -> str:
        """
        Generate a new CSRF token.
        
        Returns:
            CSRF token
        """
        return secrets.token_hex(32)
    
    def _set_csrf_cookie(self, response: Response, token: str) -> None:
        """
        Set the CSRF cookie on the response.
        
        Args:
            response: HTTP response
            token: CSRF token
        """
        response.set_cookie(
            key=self.cookie_name,
            value=token,
            max_age=self.cookie_max_age,
            secure=self.cookie_secure,
            httponly=self.cookie_httponly,
            samesite=self.cookie_samesite
        )
    
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and apply CSRF protection.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response
        """
        # Skip CSRF protection for safe methods
        if request.method in self.safe_methods:
            response = await call_next(request)
            
            # Set CSRF token cookie if it doesn't exist
            if self.cookie_name not in request.cookies:
                token = self._generate_csrf_token()
                self._set_csrf_cookie(response, token)
                
            return response
            
        # Skip CSRF protection for excluded paths
        path = request.url.path
        if any(path.startswith(excluded) for excluded in self.exclude_paths):
            return await call_next(request)
            
        # Check CSRF token
        cookie_token = request.cookies.get(self.cookie_name)
        header_token = request.headers.get(self.header_name)
        
        if not cookie_token or not header_token or cookie_token != header_token:
            return JSONResponse(
                status_code=status.HTTP_403_FORBIDDEN,
                content={"detail": "CSRF token missing or invalid"}
            )
            
        # Process the request
        response = await call_next(request)
        
        # Refresh the CSRF token
        token = self._generate_csrf_token()
        self._set_csrf_cookie(response, token)
        
        return response
