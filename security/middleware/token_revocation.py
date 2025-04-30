"""
Token revocation middleware for FastAPI applications.

This module provides middleware and utilities for token revocation and blacklisting,
enabling secure logout and token invalidation.
"""

import time
import logging
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Set, Union
from fastapi import FastAPI, Request, Response, Depends, HTTPException, status
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

# Set up logging
log = logging.getLogger(__name__)

class TokenBlacklist:
    """
    Token blacklist for storing revoked tokens.
    
    This class provides methods for adding tokens to a blacklist and checking
    if a token is blacklisted. It also handles automatic cleanup of expired tokens.
    """
    
    def __init__(self, cleanup_interval: int = 3600):
        """
        Initialize the token blacklist.
        
        Args:
            cleanup_interval: Interval in seconds for cleaning up expired tokens
        """
        self.blacklist: Dict[str, float] = {}  # token -> expiration timestamp
        self.cleanup_interval = cleanup_interval
        self.last_cleanup = time.time()
        
    def add(self, token: str, expires_at: Optional[float] = None):
        """
        Add a token to the blacklist.
        
        Args:
            token: The token to blacklist
            expires_at: Timestamp when the token expires (optional)
        """
        if expires_at is None:
            # Default to 24 hours if no expiration is provided
            expires_at = time.time() + 86400
            
        self.blacklist[token] = expires_at
        log.info(f"Token added to blacklist, expires at {datetime.fromtimestamp(expires_at)}")
        
        # Clean up expired tokens if it's time
        self._cleanup_if_needed()
        
    def is_blacklisted(self, token: str) -> bool:
        """
        Check if a token is blacklisted.
        
        Args:
            token: The token to check
            
        Returns:
            True if the token is blacklisted, False otherwise
        """
        # Clean up expired tokens if it's time
        self._cleanup_if_needed()
        
        # Check if token is in blacklist and not expired
        if token in self.blacklist:
            if time.time() > self.blacklist[token]:
                # Token has expired, remove it from blacklist
                del self.blacklist[token]
                return False
            return True
        return False
        
    def _cleanup_if_needed(self):
        """
        Clean up expired tokens if the cleanup interval has passed.
        """
        current_time = time.time()
        if current_time - self.last_cleanup > self.cleanup_interval:
            self._cleanup()
            self.last_cleanup = current_time
            
    def _cleanup(self):
        """
        Remove expired tokens from the blacklist.
        """
        current_time = time.time()
        expired_tokens = [
            token for token, expires_at in self.blacklist.items()
            if current_time > expires_at
        ]
        
        for token in expired_tokens:
            del self.blacklist[token]
            
        if expired_tokens:
            log.info(f"Cleaned up {len(expired_tokens)} expired tokens from blacklist")
            
    def clear(self):
        """
        Clear all tokens from the blacklist.
        """
        self.blacklist.clear()
        log.info("Token blacklist cleared")
        
    def __len__(self):
        """
        Get the number of tokens in the blacklist.
        
        Returns:
            Number of tokens in the blacklist
        """
        return len(self.blacklist)
        
# Global token blacklist instance
token_blacklist = TokenBlacklist()

class TokenRevocationMiddleware(BaseHTTPMiddleware):
    """
    Middleware for checking if tokens are revoked/blacklisted.
    
    This middleware checks if the token in the request is blacklisted and
    rejects the request if it is.
    """
    
    def __init__(
        self,
        app: FastAPI,
        auth_header_name: str = "Authorization",
        token_prefix: str = "Bearer ",
        exclude_paths: Optional[List[str]] = None,
    ):
        """
        Initialize the middleware.
        
        Args:
            app: The FastAPI application
            auth_header_name: Name of the authentication header
            token_prefix: Prefix for the token in the authentication header
            exclude_paths: List of paths to exclude from token checking
        """
        super().__init__(app)
        self.auth_header_name = auth_header_name
        self.token_prefix = token_prefix
        self.exclude_paths = exclude_paths or [
            "/api/auth/login",
            "/api/auth/register",
            "/api/auth/refresh",
            "/api/docs",
            "/api/openapi.json",
        ]
        
        log.info("TokenRevocationMiddleware initialized")
        
    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Process the request and check if the token is blacklisted.
        
        Args:
            request: The HTTP request
            call_next: The next middleware or route handler
            
        Returns:
            The HTTP response
        """
        try:
            # Skip token checking for excluded paths
            path = request.url.path
            if any(path.startswith(excluded) for excluded in self.exclude_paths):
                return await call_next(request)
                
            # Check if request has authorization header
            auth_header = request.headers.get(self.auth_header_name)
            if auth_header and auth_header.startswith(self.token_prefix):
                # Extract token
                token = auth_header[len(self.token_prefix):]
                
                # Check if token is blacklisted
                if token_blacklist.is_blacklisted(token):
                    log.warning(f"Blocked request with blacklisted token: {path}")
                    return JSONResponse(
                        status_code=status.HTTP_401_UNAUTHORIZED,
                        content={"detail": "Token has been revoked"},
                        headers={"WWW-Authenticate": "Bearer"}
                    )
                    
            # Process the request
            return await call_next(request)
        except Exception as e:
            log.error(f"Error in TokenRevocationMiddleware: {str(e)}")
            raise

# Dependency for revoking the current token
async def revoke_token(request: Request):
    """
    Revoke the token in the current request.
    
    This function can be used as a dependency in FastAPI routes to revoke
    the token in the current request, e.g., for logout endpoints.
    
    Args:
        request: The HTTP request
        
    Returns:
        True if the token was revoked, False otherwise
    """
    auth_header = request.headers.get("Authorization")
    if auth_header and auth_header.startswith("Bearer "):
        token = auth_header[7:]
        token_blacklist.add(token)
        log.info("Token revoked")
        return True
    return False

# Function to revoke a specific token
def revoke_specific_token(token: str, expires_at: Optional[float] = None):
    """
    Revoke a specific token.
    
    Args:
        token: The token to revoke
        expires_at: Timestamp when the token expires (optional)
        
    Returns:
        True if the token was added to the blacklist
    """
    token_blacklist.add(token, expires_at)
    return True

# Function to revoke all tokens for a user
def revoke_all_user_tokens(user_id: str, user_tokens: List[str]):
    """
    Revoke all tokens for a specific user.
    
    This function can be used to revoke all tokens for a user, e.g.,
    when changing password or when the user is deleted.
    
    Args:
        user_id: The user ID
        user_tokens: List of tokens to revoke
        
    Returns:
        Number of tokens revoked
    """
    count = 0
    for token in user_tokens:
        token_blacklist.add(token)
        count += 1
        
    log.info(f"Revoked {count} tokens for user {user_id}")
    return count
