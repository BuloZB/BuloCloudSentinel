"""
API CSRF protection middleware for FastAPI.

This module provides middleware for protecting API routes against Cross-Site Request Forgery (CSRF) attacks
using token-based protection that works well with API clients.
"""

import secrets
import time
import hmac
import hashlib
from typing import Callable, Dict, List, Optional, Set, Tuple
from datetime import datetime, timedelta

from fastapi import FastAPI, Request, Response, HTTPException, status
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

class APICsrfMiddleware(BaseHTTPMiddleware):
    """
    Middleware for API CSRF protection.
    
    This middleware implements token-based CSRF protection for API routes.
    It uses a combination of custom headers and token validation to protect against CSRF attacks.
    """
    
    def __init__(
        self,
        app: FastAPI,
        secret_key: str,
        token_header_name: str = "X-CSRF-Token",
        token_field_name: str = "csrf_token",
        token_expiry: int = 3600,  # 1 hour
        safe_methods: Optional[Set[str]] = None,
        exclude_paths: Optional[List[str]] = None,
        include_paths: Optional[List[str]] = None,
        required_headers: Optional[List[str]] = None
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            secret_key: Secret key for token generation
            token_header_name: Name of the CSRF header
            token_field_name: Name of the CSRF token field in JSON responses
            token_expiry: Token expiry time in seconds
            safe_methods: HTTP methods that don't require CSRF protection
            exclude_paths: Paths to exclude from CSRF protection
            include_paths: Paths to include in CSRF protection (overrides exclude_paths)
            required_headers: Additional headers required for non-safe methods
        """
        super().__init__(app)
        self.secret_key = secret_key
        self.token_header_name = token_header_name
        self.token_field_name = token_field_name
        self.token_expiry = token_expiry
        self.safe_methods = safe_methods or {"GET", "HEAD", "OPTIONS"}
        self.exclude_paths = exclude_paths or []
        self.include_paths = include_paths or []
        self.required_headers = required_headers or ["X-Requested-With"]
        
    def _generate_csrf_token(self, request: Request) -> Tuple[str, str]:
        """
        Generate a new CSRF token.
        
        Args:
            request: The HTTP request
            
        Returns:
            Tuple of (token, signed_token)
        """
        # Generate a random token
        token = secrets.token_hex(16)
        
        # Get client IP and user agent for additional security
        client_ip = request.client.host if request.client else "unknown"
        user_agent = request.headers.get("user-agent", "unknown")
        
        # Create a timestamp for expiration
        timestamp = int(time.time()) + self.token_expiry
        
        # Combine token, timestamp, and client info
        message = f"{token}:{timestamp}:{client_ip}:{user_agent}"
        
        # Sign the message
        signature = hmac.new(
            self.secret_key.encode(),
            message.encode(),
            hashlib.sha256
        ).hexdigest()
        
        # Create the signed token
        signed_token = f"{token}:{timestamp}:{signature}"
        
        return token, signed_token
    
    def _verify_csrf_token(self, request: Request, token: str) -> bool:
        """
        Verify a CSRF token.
        
        Args:
            request: The HTTP request
            token: The CSRF token to verify
            
        Returns:
            True if token is valid, False otherwise
        """
        try:
            # Split the token
            parts = token.split(":")
            if len(parts) != 3:
                return False
            
            token_value, timestamp_str, signature = parts
            
            # Check if token has expired
            timestamp = int(timestamp_str)
            current_time = int(time.time())
            if timestamp < current_time:
                return False
            
            # Get client IP and user agent
            client_ip = request.client.host if request.client else "unknown"
            user_agent = request.headers.get("user-agent", "unknown")
            
            # Recreate the message
            message = f"{token_value}:{timestamp}:{client_ip}:{user_agent}"
            
            # Verify the signature
            expected_signature = hmac.new(
                self.secret_key.encode(),
                message.encode(),
                hashlib.sha256
            ).hexdigest()
            
            return hmac.compare_digest(signature, expected_signature)
        except Exception:
            return False
    
    def _should_apply_csrf(self, request: Request) -> bool:
        """
        Determine if CSRF protection should be applied to this request.
        
        Args:
            request: The HTTP request
            
        Returns:
            True if CSRF protection should be applied, False otherwise
        """
        # Skip CSRF protection for safe methods
        if request.method in self.safe_methods:
            return False
        
        path = request.url.path
        
        # Include paths take precedence
        if self.include_paths and any(path.startswith(included) for included in self.include_paths):
            return True
        
        # Skip CSRF protection for excluded paths
        if any(path.startswith(excluded) for excluded in self.exclude_paths):
            return False
        
        # Apply CSRF protection by default for non-safe methods
        return True
    
    def _check_required_headers(self, request: Request) -> bool:
        """
        Check if the request has all required headers.
        
        Args:
            request: The HTTP request
            
        Returns:
            True if all required headers are present, False otherwise
        """
        for header in self.required_headers:
            if header.lower() not in [h.lower() for h in request.headers.keys()]:
                return False
        return True
    
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and apply CSRF protection.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response
        """
        # Check if CSRF protection should be applied
        if not self._should_apply_csrf(request):
            response = await call_next(request)
            
            # For safe methods, generate and include a CSRF token in the response
            if request.method in self.safe_methods and request.url.path.startswith("/api/"):
                # Only modify JSON responses
                if response.headers.get("content-type", "").startswith("application/json"):
                    # Get the response body
                    body = b""
                    async for chunk in response.body_iterator:
                        body += chunk
                    
                    # Parse the JSON
                    import json
                    try:
                        data = json.loads(body)
                        
                        # Generate a new token
                        token, signed_token = self._generate_csrf_token(request)
                        
                        # Add the token to the response
                        if isinstance(data, dict):
                            data[self.token_field_name] = signed_token
                            
                            # Create a new response
                            new_body = json.dumps(data).encode()
                            new_response = Response(
                                content=new_body,
                                status_code=response.status_code,
                                headers=dict(response.headers),
                                media_type="application/json"
                            )
                            return new_response
                    except:
                        # If there's any error, return the original response
                        pass
            
            return response
        
        # Check required headers
        if not self._check_required_headers(request):
            return JSONResponse(
                status_code=status.HTTP_403_FORBIDDEN,
                content={"detail": "Required headers missing"}
            )
        
        # Check CSRF token
        token = request.headers.get(self.token_header_name)
        if not token or not self._verify_csrf_token(request, token):
            return JSONResponse(
                status_code=status.HTTP_403_FORBIDDEN,
                content={"detail": "CSRF token missing or invalid"}
            )
        
        # Process the request
        response = await call_next(request)
        return response