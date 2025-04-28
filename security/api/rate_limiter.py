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
AUTH_RATE_LIMIT = "5/minute"  # Reduced from 10 to 5 per minute
SENSITIVE_RATE_LIMIT = "30/minute"
ADMIN_RATE_LIMIT = "120/minute"

# Progressive rate limiting for failed authentication attempts
FAILED_AUTH_LIMITS = {
    3: "3/5minutes",    # After 3 failures: 3 attempts per 5 minutes
    5: "3/15minutes",   # After 5 failures: 3 attempts per 15 minutes
    10: "3/hour",       # After 10 failures: 3 attempts per hour
    20: "3/day"         # After 20 failures: 3 attempts per day
}

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

# Initialize progressive rate limiter for authentication
progressive_auth_limiter = ProgressiveAuthRateLimiter(FAILED_AUTH_LIMITS)

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
    Rate limit decorator for authentication endpoints with progressive rate limiting.
    
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
            
            # Check progressive rate limit for authentication
            if progressive_auth_limiter.is_rate_limited(client_ip):
                # Get current limit for error message
                current_limit = progressive_auth_limiter.get_current_limit(client_ip)
                if current_limit:
                    count, period_seconds = current_limit
                    period_str = "seconds"
                    period_value = period_seconds
                    
                    if period_seconds >= 86400:
                        period_str = "days"
                        period_value = period_seconds / 86400
                    elif period_seconds >= 3600:
                        period_str = "hours"
                        period_value = period_seconds / 3600
                    elif period_seconds >= 60:
                        period_str = "minutes"
                        period_value = period_seconds / 60
                    
                    detail = f"Too many failed attempts. Limited to {count} attempts per {period_value} {period_str}."
                else:
                    detail = "Rate limit exceeded"
                
                raise RateLimitExceeded(detail=detail)
            
            # Apply standard rate limit as a fallback
            key = f"{client_ip}:{request.url.path}"
            if limiter.is_rate_limited(key, rate_limiter_config.auth_limit):
                raise RateLimitExceeded()
            
            # Call the original function and capture the response
            try:
                response = await func(request, *args, **kwargs)
                
                # Check if this was a successful authentication
                # This is a heuristic - we assume 2xx status codes indicate success
                if hasattr(response, 'status_code') and 200 <= response.status_code < 300:
                    progressive_auth_limiter.record_success(client_ip)
                elif isinstance(response, dict) and 'access_token' in response:
                    progressive_auth_limiter.record_success(client_ip)
                
                return response
            except HTTPException as e:
                # If it's an authentication failure (401), record it
                if e.status_code == status.HTTP_401_UNAUTHORIZED:
                    progressive_auth_limiter.record_failure(client_ip)
                raise
        
        return wrapper
    
    return decorator

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

# Progressive rate limiting for failed authentication attempts
class ProgressiveAuthRateLimiter:
    """Progressive rate limiter for failed authentication attempts."""
    def __init__(self, limits: Dict[int, str] = None):
        """
        Initialize progressive auth rate limiter.
        
        Args:
            limits: Dictionary mapping failure count thresholds to rate limits
        """
        self.limits = limits or FAILED_AUTH_LIMITS
        self.failure_counts: Dict[str, int] = {}  # IP -> failure count
        self.last_failure_time: Dict[str, float] = {}  # IP -> last failure time
        self.ip_timestamps: Dict[str, List[float]] = {}  # IP -> list of timestamps
        
        # Parse rate limits
        self.parsed_limits: Dict[int, tuple] = {}
        for threshold, limit_str in self.limits.items():
            count, period = self._parse_limit(limit_str)
            self.parsed_limits[threshold] = (count, period)
    
    def _parse_limit(self, limit_str: str) -> tuple:
        """
        Parse rate limit string.
        
        Args:
            limit_str: Rate limit string (e.g. "5/minute")
            
        Returns:
            Tuple of (count, period in seconds)
        """
        count, period = limit_str.split('/')
        count = int(count)
        
        if period.endswith('second') or period.endswith('seconds'):
            period_seconds = int(period.split('second')[0].strip())
        elif period.endswith('minute') or period.endswith('minutes'):
            period_seconds = int(period.split('minute')[0].strip()) * 60
        elif period.endswith('hour') or period.endswith('hours'):
            period_seconds = int(period.split('hour')[0].strip()) * 3600
        elif period.endswith('day') or period.endswith('days'):
            period_seconds = int(period.split('day')[0].strip()) * 86400
        else:
            period_seconds = 60  # Default to 1 minute
        
        return count, period_seconds
    
    def record_failure(self, ip: str) -> None:
        """
        Record a failed authentication attempt.
        
        Args:
            ip: IP address
        """
        current_time = time.time()
        
        # Initialize if not exists
        if ip not in self.failure_counts:
            self.failure_counts[ip] = 0
        
        # Increment failure count
        self.failure_counts[ip] += 1
        self.last_failure_time[ip] = current_time
        
        # Initialize timestamps if not exists
        if ip not in self.ip_timestamps:
            self.ip_timestamps[ip] = []
        
        # Add current timestamp
        self.ip_timestamps[ip].append(current_time)
    
    def record_success(self, ip: str) -> None:
        """
        Record a successful authentication attempt.
        
        Args:
            ip: IP address
        """
        # Reset failure count after successful login
        # But keep a small count to prevent brute force attacks
        if ip in self.failure_counts and self.failure_counts[ip] > 3:
            self.failure_counts[ip] = max(0, self.failure_counts[ip] - 3)
        else:
            self.failure_counts[ip] = 0
    
    def get_current_limit(self, ip: str) -> tuple:
        """
        Get the current rate limit for an IP.
        
        Args:
            ip: IP address
            
        Returns:
            Tuple of (count, period in seconds)
        """
        if ip not in self.failure_counts:
            return None
        
        failure_count = self.failure_counts[ip]
        
        # Find the highest threshold that is less than or equal to the failure count
        applicable_thresholds = [t for t in self.parsed_limits.keys() if t <= failure_count]
        if not applicable_thresholds:
            return None
        
        threshold = max(applicable_thresholds)
        return self.parsed_limits[threshold]
    
    def is_rate_limited(self, ip: str) -> bool:
        """
        Check if an IP is rate limited based on progressive limits.
        
        Args:
            ip: IP address
            
        Returns:
            True if rate limited, False otherwise
        """
        current_time = time.time()
        
        # Get current limit
        current_limit = self.get_current_limit(ip)
        if not current_limit:
            return False
        
        count_limit, period = current_limit
        
        # Initialize timestamps if not exists
        if ip not in self.ip_timestamps:
            self.ip_timestamps[ip] = []
        
        # Remove timestamps outside period
        self.ip_timestamps[ip] = [
            ts for ts in self.ip_timestamps[ip]
            if current_time - ts <= period
        ]
        
        # Check if limit exceeded
        return len(self.ip_timestamps[ip]) >= count_limit

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
