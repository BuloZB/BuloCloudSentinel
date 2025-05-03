"""
CSRF protection module for Bulo.Cloud Sentinel.

This module provides CSRF protection for web applications to prevent
cross-site request forgery attacks.
"""

import logging
import secrets
import time
import hashlib
import hmac
import base64
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Set, Union, Callable, Tuple
from fastapi import FastAPI, Request, Response, HTTPException, status, Depends
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from security.logging.secure_logging import get_secure_logger

# Set up logging
logger = get_secure_logger("csrf_protection")

class CSRFMiddleware(BaseHTTPMiddleware):
    """
    Middleware for CSRF protection.

    This middleware implements multiple layers of CSRF protection:
    1. Double submit cookie pattern
    2. SameSite cookie attribute
    3. Token rotation
    4. HMAC-based token validation
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
        token_rotation_age: int = 3600,  # 1 hour
        safe_methods: Optional[List[str]] = None,
        exclude_paths: Optional[List[str]] = None,
        enforce_samesite: bool = True,
        double_submit: bool = True
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
            token_rotation_age: Age in seconds after which to rotate the token
            safe_methods: HTTP methods that don't require CSRF protection
            exclude_paths: Paths to exclude from CSRF protection
            enforce_samesite: Whether to enforce SameSite cookie attribute
            double_submit: Whether to use double submit cookie pattern
        """
        super().__init__(app)

        if not secret_key or len(secret_key) < 32:
            logger.warning("CSRF secret key is too short or not provided. Generating a secure random key.")
            self.secret_key = secrets.token_hex(32)
        else:
            self.secret_key = secret_key

        self.cookie_name = cookie_name
        self.header_name = header_name
        self.cookie_secure = cookie_secure
        self.cookie_httponly = cookie_httponly
        self.cookie_samesite = cookie_samesite
        self.cookie_max_age = cookie_max_age
        self.token_rotation_age = token_rotation_age
        self.safe_methods = safe_methods or ["GET", "HEAD", "OPTIONS", "TRACE"]
        self.exclude_paths = exclude_paths or []
        self.enforce_samesite = enforce_samesite
        self.double_submit = double_submit

        # Validate SameSite attribute
        if self.enforce_samesite and self.cookie_samesite.lower() not in ["strict", "lax"]:
            logger.warning(
                f"Insecure SameSite attribute: {self.cookie_samesite}. Setting to 'Lax'."
            )
            self.cookie_samesite = "Lax"

        # Log configuration
        logger.info(
            "Configuring CSRF middleware",
            extra={
                "cookie_name": self.cookie_name,
                "header_name": self.header_name,
                "cookie_secure": self.cookie_secure,
                "cookie_httponly": self.cookie_httponly,
                "cookie_samesite": self.cookie_samesite,
                "token_rotation_age": self.token_rotation_age,
                "safe_methods": self.safe_methods,
                "exclude_paths": self.exclude_paths,
                "enforce_samesite": self.enforce_samesite,
                "double_submit": self.double_submit
            }
        )

    def generate_csrf_token(self) -> Tuple[str, str]:
        """
        Generate a new CSRF token with timestamp.

        Returns:
            Tuple of (token, signed_token)
        """
        # Generate a random token
        token = secrets.token_hex(32)

        # Add timestamp for token rotation
        timestamp = int(time.time())

        # Combine token and timestamp
        token_with_timestamp = f"{token}.{timestamp}"

        # Sign the token with HMAC
        signed_token = self._sign_token(token_with_timestamp)

        return token_with_timestamp, signed_token

    def _sign_token(self, token: str) -> str:
        """
        Sign a token with HMAC.

        Args:
            token: Token to sign

        Returns:
            Signed token
        """
        # Create HMAC signature
        signature = hmac.new(
            self.secret_key.encode(),
            token.encode(),
            hashlib.sha256
        ).digest()

        # Encode signature as base64
        encoded_signature = base64.urlsafe_b64encode(signature).decode().rstrip("=")

        # Return token with signature
        return f"{token}.{encoded_signature}"

    def _verify_token(self, token: str, signed_token: str) -> bool:
        """
        Verify a token against its signed version.

        Args:
            token: Token to verify
            signed_token: Signed token

        Returns:
            True if the token is valid, False otherwise
        """
        # Sign the token
        expected_signed_token = self._sign_token(token)

        # Compare with the provided signed token
        return hmac.compare_digest(expected_signed_token, signed_token)

    def _is_token_expired(self, token: str, max_age: int) -> bool:
        """
        Check if a token is expired.

        Args:
            token: Token to check
            max_age: Maximum age in seconds

        Returns:
            True if the token is expired, False otherwise
        """
        try:
            # Extract timestamp from token
            timestamp_str = token.split(".")[1]
            timestamp = int(timestamp_str)

            # Check if token is expired
            return (time.time() - timestamp) > max_age
        except (IndexError, ValueError):
            # If token format is invalid, consider it expired
            return True

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

            # Set CSRF token cookie if it doesn't exist or needs rotation
            csrf_cookie = request.cookies.get(self.cookie_name)

            if not csrf_cookie or self._should_rotate_token(csrf_cookie):
                token, signed_token = self.generate_csrf_token()

                # Set the cookie with the signed token
                response.set_cookie(
                    key=self.cookie_name,
                    value=signed_token,
                    max_age=self.cookie_max_age,
                    httponly=self.cookie_httponly,
                    secure=self.cookie_secure,
                    samesite=self.cookie_samesite
                )

                # If using double submit, set a non-httponly cookie with the token
                if self.double_submit:
                    response.set_cookie(
                        key=f"{self.cookie_name}_value",
                        value=token,
                        max_age=self.cookie_max_age,
                        httponly=False,  # Accessible to JavaScript
                        secure=self.cookie_secure,
                        samesite=self.cookie_samesite
                    )

            return response

        # Skip CSRF protection for excluded paths
        if self.is_path_excluded(request.url.path):
            return await call_next(request)

        # Get the CSRF token from the cookie
        signed_csrf_cookie = request.cookies.get(self.cookie_name)

        # Get the CSRF token from the header
        csrf_header = request.headers.get(self.header_name)

        # If this is the first request, generate a CSRF token
        if not signed_csrf_cookie:
            response = await call_next(request)
            token, signed_token = self.generate_csrf_token()

            # Set the cookie with the signed token
            response.set_cookie(
                key=self.cookie_name,
                value=signed_token,
                max_age=self.cookie_max_age,
                httponly=self.cookie_httponly,
                secure=self.cookie_secure,
                samesite=self.cookie_samesite
            )

            # If using double submit, set a non-httponly cookie with the token
            if self.double_submit:
                response.set_cookie(
                    key=f"{self.cookie_name}_value",
                    value=token,
                    max_age=self.cookie_max_age,
                    httponly=False,  # Accessible to JavaScript
                    secure=self.cookie_secure,
                    samesite=self.cookie_samesite
                )

            return response

        # Extract the token from the signed cookie
        try:
            token_parts = signed_csrf_cookie.split(".")
            if len(token_parts) < 3:
                raise ValueError("Invalid token format")

            token = f"{token_parts[0]}.{token_parts[1]}"
        except (ValueError, IndexError):
            logger.warning(
                "Invalid CSRF token format",
                extra={
                    "path": request.url.path,
                    "method": request.method
                }
            )

            return Response(
                content="Invalid CSRF token format",
                status_code=status.HTTP_403_FORBIDDEN,
                media_type="text/plain",
            )

        # Verify the CSRF token
        is_valid = False

        # Check if the token is expired
        if self._is_token_expired(token, self.cookie_max_age):
            logger.warning(
                "CSRF token expired",
                extra={
                    "path": request.url.path,
                    "method": request.method
                }
            )
        # Verify the token signature
        elif not self._verify_token(token, signed_csrf_cookie):
            logger.warning(
                "CSRF token signature invalid",
                extra={
                    "path": request.url.path,
                    "method": request.method
                }
            )
        # Verify the token against the header
        elif not csrf_header:
            logger.warning(
                "CSRF token header missing",
                extra={
                    "path": request.url.path,
                    "method": request.method
                }
            )
        # If using double submit, verify against the header
        elif self.double_submit:
            # The header should match the token (not the signed token)
            is_valid = hmac.compare_digest(token, csrf_header)

            if not is_valid:
                logger.warning(
                    "CSRF token header does not match token",
                    extra={
                        "path": request.url.path,
                        "method": request.method
                    }
                )
        # Otherwise, verify that the header matches the signed token
        else:
            is_valid = hmac.compare_digest(signed_csrf_cookie, csrf_header)

            if not is_valid:
                logger.warning(
                    "CSRF token header does not match signed token",
                    extra={
                        "path": request.url.path,
                        "method": request.method
                    }
                )

        # If the token is invalid, return an error
        if not is_valid:
            return Response(
                content="CSRF token missing or invalid",
                status_code=status.HTTP_403_FORBIDDEN,
                media_type="text/plain",
            )

        # Continue with the request
        response = await call_next(request)

        # Rotate the token if needed
        if self._should_rotate_token(token):
            new_token, new_signed_token = self.generate_csrf_token()

            # Set the cookie with the new signed token
            response.set_cookie(
                key=self.cookie_name,
                value=new_signed_token,
                max_age=self.cookie_max_age,
                httponly=self.cookie_httponly,
                secure=self.cookie_secure,
                samesite=self.cookie_samesite
            )

            # If using double submit, set a non-httponly cookie with the new token
            if self.double_submit:
                response.set_cookie(
                    key=f"{self.cookie_name}_value",
                    value=new_token,
                    max_age=self.cookie_max_age,
                    httponly=False,  # Accessible to JavaScript
                    secure=self.cookie_secure,
                    samesite=self.cookie_samesite
                )

        return response

    def _should_rotate_token(self, token: str) -> bool:
        """
        Check if a token should be rotated.

        Args:
            token: Token to check

        Returns:
            True if the token should be rotated, False otherwise
        """
        try:
            # Extract timestamp from token
            if "." not in token:
                return True

            timestamp_str = token.split(".")[1]
            timestamp = int(timestamp_str)

            # Check if token should be rotated
            return (time.time() - timestamp) > self.token_rotation_age
        except (IndexError, ValueError):
            # If token format is invalid, it should be rotated
            return True

def add_csrf_protection(
    app: FastAPI,
    secret_key: str,
    cookie_name: str = "csrf_token",
    header_name: str = "X-CSRF-Token",
    cookie_secure: bool = True,
    cookie_httponly: bool = True,
    cookie_samesite: str = "Lax",
    cookie_max_age: int = 86400,  # 24 hours
    token_rotation_age: int = 3600,  # 1 hour
    safe_methods: Optional[List[str]] = None,
    exclude_paths: Optional[List[str]] = None,
    enforce_samesite: bool = True,
    double_submit: bool = True
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
        token_rotation_age: Age in seconds after which to rotate the token
        safe_methods: HTTP methods that don't require CSRF protection
        exclude_paths: Paths to exclude from CSRF protection
        enforce_samesite: Whether to enforce SameSite cookie attribute
        double_submit: Whether to use double submit cookie pattern
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
        token_rotation_age=token_rotation_age,
        safe_methods=safe_methods,
        exclude_paths=exclude_paths,
        enforce_samesite=enforce_samesite,
        double_submit=double_submit
    )

    logger.info("Added CSRF protection middleware to FastAPI application")

def csrf_protect(
    request: Request,
    csrf_cookie_name: str = "csrf_token",
    csrf_header_name: str = "X-CSRF-Token",
    double_submit: bool = True
) -> None:
    """
    Verify CSRF protection for a request.

    This function can be used as a dependency in FastAPI routes.

    Args:
        request: FastAPI request object
        csrf_cookie_name: Name of the CSRF cookie
        csrf_header_name: Name of the CSRF header
        double_submit: Whether to use double submit cookie pattern

    Raises:
        HTTPException: If CSRF protection fails
    """
    # Get the CSRF token from the cookie
    signed_csrf_cookie = request.cookies.get(csrf_cookie_name)

    # Get the CSRF token from the header
    csrf_header = request.headers.get(csrf_header_name)

    # Verify the CSRF token
    if not signed_csrf_cookie or not csrf_header:
        logger.warning(
            "CSRF protection failed: missing token",
            extra={
                "path": request.url.path,
                "method": request.method,
                "has_cookie": bool(signed_csrf_cookie),
                "has_header": bool(csrf_header)
            }
        )

        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="CSRF protection failed: missing token"
        )

    # Extract the token from the signed cookie
    try:
        token_parts = signed_csrf_cookie.split(".")
        if len(token_parts) < 3:
            raise ValueError("Invalid token format")

        token = f"{token_parts[0]}.{token_parts[1]}"
    except (ValueError, IndexError):
        logger.warning(
            "CSRF protection failed: invalid token format",
            extra={
                "path": request.url.path,
                "method": request.method
            }
        )

        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="CSRF protection failed: invalid token format"
        )

    # If using double submit, verify against the token
    if double_submit:
        # The header should match the token (not the signed token)
        if not hmac.compare_digest(token, csrf_header):
            logger.warning(
                "CSRF protection failed: token mismatch",
                extra={
                    "path": request.url.path,
                    "method": request.method
                }
            )

            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="CSRF protection failed: token mismatch"
            )
    # Otherwise, verify that the header matches the signed token
    else:
        if not hmac.compare_digest(signed_csrf_cookie, csrf_header):
            logger.warning(
                "CSRF protection failed: token mismatch",
                extra={
                    "path": request.url.path,
                    "method": request.method
                }
            )

            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="CSRF protection failed: token mismatch"
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
