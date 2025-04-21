"""
Rate limiter for Bulo.Cloud Sentinel Security Module.

This module provides rate limiting for API endpoints to prevent abuse.
"""

import time
from typing import Callable, Dict, Optional, Union, List

from fastapi import Depends, HTTPException, Request, status
from fastapi.security import OAuth2PasswordBearer
from slowapi import Limiter
from slowapi.util import get_remote_address

# OAuth2 scheme for token extraction
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Rate limit configurations
DEFAULT_RATE_LIMIT = "60/minute"
AUTH_RATE_LIMIT = "10/minute"
SENSITIVE_RATE_LIMIT = "30/minute"
ADMIN_RATE_LIMIT = "120/minute"

# Rate limit storage
class RateLimitExceeded(HTTPException):
    """Exception raised when rate limit is exceeded."""
    def __init__(self, detail: str = "Rate limit exceeded"):
        super().__init__(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=detail,
            headers={"Retry-After": "60"}
        )

class RateLimiterConfig:
    """Rate limiter configuration."""
    def __init__(
        self,
        default_limit: str = DEFAULT_RATE_LIMIT,
        auth_limit: str = AUTH_RATE_LIMIT,
        sensitive_limit: str = SENSITIVE_RATE_LIMIT,
        admin_limit: str = ADMIN_RATE_LIMIT,
        whitelist_ips: Optional[List[str]] = None,
        blacklist_ips: Optional[List[str]] = None
    ):
        self.default_limit = default_limit
        self.auth_limit = auth_limit
        self.sensitive_limit = sensitive_limit
        self.admin_limit = admin_limit
        self.whitelist_ips = whitelist_ips or []
        self.blacklist_ips = blacklist_ips or []

# Global configuration
rate_limiter_config = RateLimiterConfig()

def configure_rate_limiter(config: RateLimiterConfig):
    """
    Configure the rate limiter.
    
    Args:
        config: Rate limiter configuration
    """
    global rate_limiter_config
    rate_limiter_config = config

def rate_limit(limit: Optional[str] = None):
    """
    Rate limit decorator for FastAPI endpoints.
    
    Args:
        limit: Rate limit string (e.g. "60/minute")
        
    Returns:
        Decorator function
    """
    def decorator(func: Callable):
        async def wrapper(request: Request, *args, **kwargs):
            # Get client IP
            client_ip = get_remote_address(request)
            
            # Check if IP is whitelisted
            if client_ip in rate_limiter_config.whitelist_ips:
                return await func(request, *args, **kwargs)
            
            # Check if IP is blacklisted
            if client_ip in rate_limiter_config.blacklist_ips:
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="Access denied"
                )
            
            # Get rate limit
            rate_limit_value = limit or rate_limiter_config.default_limit
            
            # Check rate limit
            key = f"{client_ip}:{request.url.path}"
            
            # Use slowapi limiter
            if not limiter.is_rate_limited(key, rate_limit_value):
                return await func(request, *args, **kwargs)
            else:
                raise RateLimitExceeded()
        
        return wrapper
    
    return decorator

def auth_rate_limit():
    """
    Rate limit decorator for authentication endpoints.
    
    Returns:
        Decorator function
    """
    return rate_limit(rate_limiter_config.auth_limit)

def sensitive_rate_limit():
    """
    Rate limit decorator for sensitive endpoints.
    
    Returns:
        Decorator function
    """
    return rate_limit(rate_limiter_config.sensitive_limit)

def admin_rate_limit():
    """
    Rate limit decorator for admin endpoints.
    
    Returns:
        Decorator function
    """
    return rate_limit(rate_limiter_config.admin_limit)

# IP-based rate limiting
class IPRateLimiter:
    """IP-based rate limiter."""
    def __init__(self, limit: int, window: int = 60):
        """
        Initialize IP rate limiter.
        
        Args:
            limit: Maximum number of requests
            window: Time window in seconds
        """
        self.limit = limit
        self.window = window
        self.ip_timestamps: Dict[str, List[float]] = {}
    
    def is_rate_limited(self, ip: str) -> bool:
        """
        Check if an IP is rate limited.
        
        Args:
            ip: IP address
            
        Returns:
            True if rate limited, False otherwise
        """
        current_time = time.time()
        
        # Initialize timestamps for IP if not exists
        if ip not in self.ip_timestamps:
            self.ip_timestamps[ip] = []
        
        # Remove timestamps outside window
        self.ip_timestamps[ip] = [
            ts for ts in self.ip_timestamps[ip]
            if current_time - ts <= self.window
        ]
        
        # Check if limit exceeded
        if len(self.ip_timestamps[ip]) >= self.limit:
            return True
        
        # Add current timestamp
        self.ip_timestamps[ip].append(current_time)
        
        return False

# User-based rate limiting
class UserRateLimiter:
    """User-based rate limiter."""
    def __init__(self, limit: int, window: int = 60):
        """
        Initialize user rate limiter.
        
        Args:
            limit: Maximum number of requests
            window: Time window in seconds
        """
        self.limit = limit
        self.window = window
        self.user_timestamps: Dict[str, List[float]] = {}
    
    def is_rate_limited(self, user_id: str) -> bool:
        """
        Check if a user is rate limited.
        
        Args:
            user_id: User ID
            
        Returns:
            True if rate limited, False otherwise
        """
        current_time = time.time()
        
        # Initialize timestamps for user if not exists
        if user_id not in self.user_timestamps:
            self.user_timestamps[user_id] = []
        
        # Remove timestamps outside window
        self.user_timestamps[user_id] = [
            ts for ts in self.user_timestamps[user_id]
            if current_time - ts <= self.window
        ]
        
        # Check if limit exceeded
        if len(self.user_timestamps[user_id]) >= self.limit:
            return True
        
        # Add current timestamp
        self.user_timestamps[user_id].append(current_time)
        
        return False
