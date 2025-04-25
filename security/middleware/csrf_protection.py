"""
CSRF protection middleware for FastAPI.

This middleware adds CSRF protection to HTTP requests.
"""

import hashlib
import hmac
import secrets
import time
from typing import Callable, Dict, List, Optional, Set, Tuple

from fastapi import FastAPI, Request, Response, HTTPException
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

class CSRFProtectionMiddleware(BaseHTTPMiddleware):
    """
    Middleware for CSRF protection.
    
    This middleware adds CSRF protection to HTTP requests.
    """
    
    def __init__(
        self,
        app: FastAPI,
        secret_key: str,
        cookie_name: str = "csrf_token",
        header_name: str = "X-CSRF-Token",
        cookie_secure: bool = True,
        cookie_httponly: bool = True,
        cookie_samesite: str = "Lax",
        cookie_max_age: int = 3600,
        safe_methods: Optional[List[str]] = None,
        exclude_paths: Optional[List[str]] = None,
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            secret_key: Secret key for CSRF token generation
            cookie_name: Name of the CSRF cookie
            header_name: Name of the CSRF header
            cookie_secure: Whether the cookie should be secure
            cookie_httponly: Whether the cookie should be HTTP-only
            cookie_samesite: SameSite attribute for the cookie
            cookie_max_age: Max age of the cookie in seconds
            safe_methods: List of HTTP methods that don't require CSRF protection
            exclude_paths: List of paths to exclude from CSRF protection
        """
        super().__init__(app)
        self.secret_key = secret_key
        self.cookie_name = cookie_name
        self.header_name = header_name
        self.cookie_secure = cookie_secure
        self.cookie_httponly = cookie_httponly
        self.cookie_samesite = cookie_samesite
        self.cookie_max_age = cookie_max_age
        self.safe_methods = safe_methods or ["GET", "HEAD", "OPTIONS"]
        self.exclude_paths = exclude_paths or []
    
    def _generate_csrf_token(self) -> Tuple[str, str]:
        """
        Generate a CSRF token.
        
        Returns:
            Tuple of (token, signed_token)
        """
        # Generate a random token
        token = secrets.token_hex(32)
        
        # Sign the token
        timestamp = str(int(time.time()))
        msg = f"{token}:{timestamp}"
        signature = hmac.new(
            self.secret_key.encode(),
            msg.encode(),
            digestmod=hashlib.sha256
        ).hexdigest()
        
        # Create the signed token
        signed_token = f"{token}:{timestamp}:{signature}"
        
        return token, signed_token
    
    def _verify_csrf_token(self, token: str, signed_token: str) -> bool:
        """
        Verify a CSRF token.
        
        Args:
            token: The CSRF token
            signed_token: The signed CSRF token
            
        Returns:
            True if the token is valid, False otherwise
        """
        # Parse the signed token
        try:
            parts = signed_token.split(":")
            if len(parts) != 3:
                return False
            
            stored_token, timestamp, signature = parts
            
            # Verify the token
            if token != stored_token:
                return False
            
            # Verify the signature
            msg = f"{stored_token}:{timestamp}"
            expected_signature = hmac.new(
                self.secret_key.encode(),
                msg.encode(),
                digestmod=hashlib.sha256
            ).hexdigest()
            
            if not hmac.compare_digest(signature, expected_signature):
                return False
            
            # Verify the timestamp (optional)
            # current_time = int(time.time())
            # token_time = int(timestamp)
            # if current_time - token_time > self.cookie_max_age:
            #     return False
            
            return True
        except Exception:
            return False
    
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and add CSRF protection.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response
        """
        # Skip CSRF protection for excluded paths
        path = request.url.path
        if any(path.startswith(excluded) for excluded in self.exclude_paths):
            return await call_next(request)
        
        # Skip CSRF protection for safe methods
        if request.method in self.safe_methods:
            # For GET requests, set a new CSRF token if not present
            if request.method == "GET" and self.cookie_name not in request.cookies:
                token, signed_token = self._generate_csrf_token()
                response = await call_next(request)
                
                # Set the CSRF cookie
                response.set_cookie(
                    key=self.cookie_name,
                    value=signed_token,
                    secure=self.cookie_secure,
                    httponly=self.cookie_httponly,
                    samesite=self.cookie_samesite,
                    max_age=self.cookie_max_age
                )
                
                return response
            else:
                return await call_next(request)
        
        # For unsafe methods, verify the CSRF token
        csrf_cookie = request.cookies.get(self.cookie_name)
        csrf_header = request.headers.get(self.header_name)
        
        if not csrf_cookie or not csrf_header:
            return JSONResponse(
                status_code=403,
                content={"detail": "CSRF token missing"}
            )
        
        if not self._verify_csrf_token(csrf_header, csrf_cookie):
            return JSONResponse(
                status_code=403,
                content={"detail": "CSRF token invalid"}
            )
        
        # Process the request
        response = await call_next(request)
        
        return response
