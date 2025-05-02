"""
CSRF protection module for Bulo.Cloud Sentinel.

This module provides CSRF protection for web applications to prevent
cross-site request forgery attacks.
"""

import logging
import secrets
import time
from typing import Dict, List, Optional, Set, Union, Callable
from fastapi import FastAPI, Request, Response, HTTPException, status
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from security.logging.secure_logging import get_secure_logger

# Set up logging
logger = get_secure_logger("csrf_protection")

class CSRFMiddleware(BaseHTTPMiddleware):
    """
    Middleware for CSRF protection.
    """
    
    def __init__(
        self,
        app: ASGIApp,
        secret_key: str,
        cookie_name: str = "csrf_token",
        header_name: str = "X-CSRF-Token",
        cookie_secure: bool = True,
        cookie_httponly: bool = True,
        cookie_samesite: str = "Lax",
        cookie_max_age: int = 86400,  # 24 hours
        safe_methods: Optional[List[str]] = None,
        exclude_paths: Optional[List[str]] = None
    ):
        """
        Initialize the CSRF middleware.
        
        Args:
            app: ASGI application
            secret_key: Secret key for token generation
            cookie_name: Name of the CSRF cookie
            header_name: Name of the CSRF header
            cookie_secure: Whether to set the Secure flag on the cookie
            cookie_httponly: Whether to set the HttpOnly flag on the cookie
            cookie_samesite: SameSite attribute for the cookie
            cookie_max_age: Maximum age of the cookie in seconds
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
        self.safe_methods = safe_methods or ["GET", "HEAD", "OPTIONS", "TRACE"]
        self.exclude_paths = exclude_paths or []
        
        # Log configuration
        logger.info(
            "Configuring CSRF middleware",
            extra={
                "cookie_name": self.cookie_name,
                "header_name": self.header_name,
                "cookie_secure": self.cookie_secure,
                "cookie_httponly": self.cookie_httponly,
                "cookie_samesite": self.cookie_samesite,
                "safe_methods": self.safe_methods,
                "exclude_paths": self.exclude_paths
            }
        )
    
    def generate_csrf_token(self) -> str:
        """
        Generate a new CSRF token.
        
        Returns:
            CSRF token
        """
        return secrets.token_hex(32)
    
    def is_path_excluded(self, path: str) -> bool:
        """
        Check if a path is excluded from CSRF protection.
        
        Args:
            path: Request path
            
        Returns:
            True if the path is excluded, False otherwise
        """
        return any(path.startswith(excluded_path) for excluded_path in self.exclude_paths)
    
    async def dispatch(self, request: Request, call_next) -> Response:
        """
        Dispatch the request and handle CSRF protection.
        
        Args:
            request: Request object
            call_next: Next middleware or route handler
            
        Returns:
            Response
        """
        # Skip CSRF protection for safe methods
        if request.method in self.safe_methods:
            response = await call_next(request)
            
            # Set CSRF token cookie if it doesn't exist
            if self.cookie_name not in request.cookies:
                csrf_token = self.generate_csrf_token()
                response.set_cookie(
                    key=self.cookie_name,
                    value=csrf_token,
                    max_age=self.cookie_max_age,
                    httponly=self.cookie_httponly,
                    secure=self.cookie_secure,
                    samesite=self.cookie_samesite
                )
            
            return response
        
        # Skip CSRF protection for excluded paths
        if self.is_path_excluded(request.url.path):
            return await call_next(request)
        
        # Get the CSRF token from the cookie
        csrf_cookie = request.cookies.get(self.cookie_name)
        
        # Get the CSRF token from the header
        csrf_header = request.headers.get(self.header_name)
        
        # If this is the first request, generate a CSRF token
        if not csrf_cookie:
            response = await call_next(request)
            csrf_token = self.generate_csrf_token()
            response.set_cookie(
                key=self.cookie_name,
                value=csrf_token,
                max_age=self.cookie_max_age,
                httponly=self.cookie_httponly,
                secure=self.cookie_secure,
                samesite=self.cookie_samesite
            )
            return response
        
        # Verify the CSRF token
        if not csrf_header or csrf_cookie != csrf_header:
            logger.warning(
                "CSRF token missing or invalid",
                extra={
                    "path": request.url.path,
                    "method": request.method,
                    "has_cookie": bool(csrf_cookie),
                    "has_header": bool(csrf_header),
                    "tokens_match": csrf_cookie == csrf_header if csrf_cookie and csrf_header else False
                }
            )
            
            return Response(
                content="CSRF token missing or invalid",
                status_code=status.HTTP_403_FORBIDDEN,
                media_type="text/plain",
            )
        
        # Continue with the request
        response = await call_next(request)
        return response

def add_csrf_protection(
    app: FastAPI,
    secret_key: str,
    cookie_name: str = "csrf_token",
    header_name: str = "X-CSRF-Token",
    cookie_secure: bool = True,
    cookie_httponly: bool = True,
    cookie_samesite: str = "Lax",
    cookie_max_age: int = 86400,  # 24 hours
    safe_methods: Optional[List[str]] = None,
    exclude_paths: Optional[List[str]] = None
) -> None:
    """
    Add CSRF protection middleware to a FastAPI application.
    
    Args:
        app: FastAPI application
        secret_key: Secret key for token generation
        cookie_name: Name of the CSRF cookie
        header_name: Name of the CSRF header
        cookie_secure: Whether to set the Secure flag on the cookie
        cookie_httponly: Whether to set the HttpOnly flag on the cookie
        cookie_samesite: SameSite attribute for the cookie
        cookie_max_age: Maximum age of the cookie in seconds
        safe_methods: HTTP methods that don't require CSRF protection
        exclude_paths: Paths to exclude from CSRF protection
    """
    # Add CSRF middleware
    app.add_middleware(
        CSRFMiddleware,
        secret_key=secret_key,
        cookie_name=cookie_name,
        header_name=header_name,
        cookie_secure=cookie_secure,
        cookie_httponly=cookie_httponly,
        cookie_samesite=cookie_samesite,
        cookie_max_age=cookie_max_age,
        safe_methods=safe_methods,
        exclude_paths=exclude_paths
    )
    
    logger.info("Added CSRF protection middleware to FastAPI application")

def csrf_protect(
    request: Request,
    csrf_cookie_name: str = "csrf_token",
    csrf_header_name: str = "X-CSRF-Token"
) -> None:
    """
    Verify CSRF protection for a request.
    
    This function can be used as a dependency in FastAPI routes.
    
    Args:
        request: FastAPI request object
        csrf_cookie_name: Name of the CSRF cookie
        csrf_header_name: Name of the CSRF header
        
    Raises:
        HTTPException: If CSRF protection fails
    """
    # Get the CSRF token from the cookie
    csrf_cookie = request.cookies.get(csrf_cookie_name)
    
    # Get the CSRF token from the header
    csrf_header = request.headers.get(csrf_header_name)
    
    # Verify the CSRF token
    if not csrf_cookie or not csrf_header or csrf_cookie != csrf_header:
        logger.warning(
            "CSRF protection failed",
            extra={
                "path": request.url.path,
                "method": request.method,
                "has_cookie": bool(csrf_cookie),
                "has_header": bool(csrf_header),
                "tokens_match": csrf_cookie == csrf_header if csrf_cookie and csrf_header else False
            }
        )
        
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="CSRF protection failed"
        )

def get_csrf_token(request: Request, csrf_cookie_name: str = "csrf_token") -> str:
    """
    Get the CSRF token from a request.
    
    Args:
        request: FastAPI request object
        csrf_cookie_name: Name of the CSRF cookie
        
    Returns:
        CSRF token
        
    Raises:
        HTTPException: If the CSRF token is not found
    """
    # Get the CSRF token from the cookie
    csrf_token = request.cookies.get(csrf_cookie_name)
    
    if not csrf_token:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="CSRF token not found"
        )
    
    return csrf_token

def set_csrf_token(
    response: Response,
    csrf_token: str,
    csrf_cookie_name: str = "csrf_token",
    cookie_secure: bool = True,
    cookie_httponly: bool = True,
    cookie_samesite: str = "Lax",
    cookie_max_age: int = 86400  # 24 hours
) -> None:
    """
    Set the CSRF token in a response.
    
    Args:
        response: FastAPI response object
        csrf_token: CSRF token
        csrf_cookie_name: Name of the CSRF cookie
        cookie_secure: Whether to set the Secure flag on the cookie
        cookie_httponly: Whether to set the HttpOnly flag on the cookie
        cookie_samesite: SameSite attribute for the cookie
        cookie_max_age: Maximum age of the cookie in seconds
    """
    response.set_cookie(
        key=csrf_cookie_name,
        value=csrf_token,
        max_age=cookie_max_age,
        httponly=cookie_httponly,
        secure=cookie_secure,
        samesite=cookie_samesite
    )
