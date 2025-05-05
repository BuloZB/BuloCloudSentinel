"""
Rate limiting utilities for Bulo.Cloud Sentinel.

This module provides functions for implementing rate limiting
to protect API endpoints from abuse.
"""

import time
import hashlib
from typing import Dict, List, Optional, Tuple, Callable
from datetime import datetime

from fastapi import FastAPI, Request, Response, status, Depends
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware

try:
    import redis
    REDIS_AVAILABLE = True
except ImportError:
    REDIS_AVAILABLE = False

from ..logging.secure_logging import get_secure_logger


class RateLimiter:
    """
    Rate limiter for API endpoints.
    """

    def __init__(
        self,
        limit: int = 100,
        window: int = 60,
        block_duration: int = 300,  # 5 minutes block by default
        key_func: Optional[Callable[[Request], str]] = None,
        storage: Optional[Dict[str, List[float]]] = None,
        redis_url: Optional[str] = None,
        default_limit: int = 100,
        default_window: int = 60,
        default_block_duration: int = 300
    ):
        """
        Initialize the rate limiter.

        Args:
            limit: Maximum number of requests per window
            window: Time window in seconds
            block_duration: Duration to block IPs that exceed the rate limit
            key_func: Function to extract key from request
            storage: Storage for request timestamps
            redis_url: Redis connection URL for distributed rate limiting
            default_limit: Default limit for endpoints without specific limits
            default_window: Default window for endpoints without specific windows
            default_block_duration: Default block duration for endpoints
        """
        self.limit = limit
        self.window = window
        self.block_duration = block_duration
        self.key_func = key_func or self._default_key_func
        self.storage = storage or {}
        self.blocked_ips = {}  # Store blocked IPs with expiration time
        self.default_limit = default_limit
        self.default_window = default_window
        self.default_block_duration = default_block_duration
        self.logger = get_secure_logger("rate_limiter")

        # Initialize Redis if URL is provided and Redis is available
        self.redis = None
        if redis_url and REDIS_AVAILABLE:
            try:
                self.redis = redis.from_url(redis_url)
                self.logger.info(f"Connected to Redis at {redis_url}")
            except Exception as e:
                self.logger.error(f"Failed to connect to Redis: {str(e)}")
                # Fall back to in-memory storage
                self.redis = None

    def _default_key_func(self, request: Request) -> str:
        """
        Default function to extract key from request.

        Args:
            request: Request object

        Returns:
            Key for rate limiting
        """
        # Use client IP as key
        client_ip = request.client.host if request.client else "unknown"

        # Add path to make it more specific
        path = request.url.path

        # Hash the key to avoid storing sensitive information
        # Use SHA-256 instead of MD5 for better security
        key = hashlib.sha256(f"{client_ip}:{path}".encode()).hexdigest()

        return key

    def is_rate_limited(self, request: Request, custom_limit: Optional[int] = None,
                       custom_window: Optional[int] = None) -> Tuple[bool, int, int]:
        """
        Check if a request is rate limited.

        Args:
            request: Request object
            custom_limit: Optional custom limit for this request
            custom_window: Optional custom window for this request

        Returns:
            Tuple of (is_limited, remaining, reset)
        """
        # Use custom limit/window if provided, otherwise use defaults
        limit = custom_limit or self.limit
        window = custom_window or self.window

        # Get client IP
        client_ip = request.client.host if request.client else "unknown"

        # Check if IP is blocked
        if client_ip in self.blocked_ips:
            block_until = self.blocked_ips[client_ip]
            now = time.time()

            if now < block_until:
                # IP is still blocked
                reset_time = int(block_until)
                return True, 0, reset_time
            else:
                # Block has expired, remove from blocked IPs
                del self.blocked_ips[client_ip]

        # Generate key for rate limiting
        key = self.key_func(request)
        now = time.time()

        # Use Redis if available
        if self.redis:
            return self._check_redis_rate_limit(key, now, limit, window)
        else:
            return self._check_memory_rate_limit(key, now, limit, window, client_ip)

    def _check_redis_rate_limit(self, key: str, now: float, limit: int, window: int) -> Tuple[bool, int, int]:
        """
        Check rate limit using Redis.

        Args:
            key: Rate limiting key
            now: Current timestamp
            limit: Request limit
            window: Time window

        Returns:
            Tuple of (is_limited, remaining, reset)
        """
        # Use Redis sorted set to store timestamps
        redis_key = f"rate_limit:{key}"

        # Remove timestamps outside the window
        window_start = now - window
        self.redis.zremrangebyscore(redis_key, 0, window_start)

        # Count requests in the current window
        num_requests = self.redis.zcard(redis_key)

        # Check if limit is exceeded
        is_limited = num_requests >= limit

        # Add current timestamp if not limited
        if not is_limited:
            self.redis.zadd(redis_key, {str(now): now})
            # Set expiration on the key
            self.redis.expire(redis_key, window * 2)

        # Calculate remaining requests and reset time
        remaining = max(0, limit - num_requests)
        reset = int(window_start + window)

        return is_limited, remaining, reset

    def _check_memory_rate_limit(self, key: str, now: float, limit: int, window: int,
                                client_ip: str) -> Tuple[bool, int, int]:
        """
        Check rate limit using in-memory storage.

        Args:
            key: Rate limiting key
            now: Current timestamp
            limit: Request limit
            window: Time window
            client_ip: Client IP address

        Returns:
            Tuple of (is_limited, remaining, reset)
        """
        # Get request timestamps for this key
        timestamps = self.storage.get(key, [])

        # Remove timestamps outside the window
        window_start = now - window
        timestamps = [ts for ts in timestamps if ts > window_start]

        # Check if limit is exceeded
        is_limited = len(timestamps) >= limit

        # Add current timestamp if not limited
        if not is_limited:
            timestamps.append(now)
        else:
            # Block the IP if it exceeds the limit by a significant amount
            # This helps prevent brute force attacks
            if len(timestamps) >= limit * 2:
                self.block_ip(client_ip)

        # Update storage
        self.storage[key] = timestamps

        # Calculate remaining requests and reset time
        remaining = max(0, limit - len(timestamps))
        reset = int(window_start + window)

        return is_limited, remaining, reset

    def block_ip(self, ip: str, duration: Optional[int] = None) -> None:
        """
        Block an IP address for a specified duration.

        Args:
            ip: IP address to block
            duration: Duration in seconds to block the IP
        """
        block_duration = duration or self.block_duration
        block_until = time.time() + block_duration

        self.blocked_ips[ip] = block_until

        self.logger.warning(
            f"Blocked IP {ip} for {block_duration} seconds",
            {"ip": ip, "duration": block_duration, "until": datetime.fromtimestamp(block_until).isoformat()}
        )

        # If Redis is available, store the block in Redis for distributed blocking
        if self.redis:
            try:
                self.redis.setex(f"ip_block:{ip}", block_duration, "1")
            except Exception as e:
                self.logger.error(f"Failed to store IP block in Redis: {str(e)}")

    def __call__(self, request: Request) -> Tuple[bool, int, int]:
        """
        Check if a request is rate limited.

        Args:
            request: Request object

        Returns:
            Tuple of (is_limited, remaining, reset)
        """
        return self.is_rate_limited(request)


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Middleware for rate limiting.
    """

    def __init__(
        self,
        app: FastAPI,
        rate_limiter: Optional[RateLimiter] = None,
        limit: int = 100,
        window: int = 60,
        block_duration: int = 300,
        key_func: Optional[Callable[[Request], str]] = None,
        exclude_paths: Optional[List[str]] = None,
        storage: Optional[Dict[str, List[float]]] = None,
        redis_url: Optional[str] = None,
        error_message: str = "Rate limit exceeded",
        retry_after: bool = True
    ):
        """
        Initialize the rate limit middleware.

        Args:
            app: FastAPI application
            rate_limiter: Optional existing RateLimiter instance
            limit: Maximum number of requests per window
            window: Time window in seconds
            block_duration: Duration to block IPs that exceed the rate limit
            key_func: Function to extract key from request
            exclude_paths: Paths to exclude from rate limiting
            storage: Storage for request timestamps
            redis_url: Redis connection URL for distributed rate limiting
            error_message: Error message for rate limited requests
            retry_after: Whether to include Retry-After header
        """
        super().__init__(app)

        # Use provided rate limiter or create a new one
        if rate_limiter:
            self.limiter = rate_limiter
        else:
            self.limiter = RateLimiter(
                limit=limit,
                window=window,
                block_duration=block_duration,
                key_func=key_func,
                storage=storage,
                redis_url=redis_url
            )

        self.exclude_paths = exclude_paths or []
        self.error_message = error_message
        self.retry_after = retry_after
        self.logger = get_secure_logger("rate_limit_middleware")

    async def dispatch(self, request: Request, call_next):
        """
        Process a request.

        Args:
            request: Request object
            call_next: Function to call next middleware

        Returns:
            Response
        """
        # Skip rate limiting for excluded paths
        if any(request.url.path.startswith(path) for path in self.exclude_paths):
            return await call_next(request)

        # Get client IP for logging
        client_ip = request.client.host if request.client else "unknown"

        # Check rate limit
        is_limited, remaining, reset = self.limiter(request)

        # Set rate limit headers
        headers = {
            "X-RateLimit-Limit": str(self.limiter.limit),
            "X-RateLimit-Remaining": str(remaining),
            "X-RateLimit-Reset": str(reset)
        }

        # Add Retry-After header if enabled
        if self.retry_after and is_limited:
            retry_after = max(1, reset - int(time.time()))
            headers["Retry-After"] = str(retry_after)

        # Return error response if rate limited
        if is_limited:
            # Log the rate limit event
            self.logger.warning(
                "Rate limit exceeded",
                {
                    "client": client_ip,
                    "path": request.url.path,
                    "method": request.method,
                    "reset": datetime.fromtimestamp(reset).isoformat()
                }
            )

            # Return 429 Too Many Requests response
            return JSONResponse(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                content={
                    "detail": self.error_message,
                    "retry_after": reset - int(time.time())
                },
                headers=headers
            )

        # Process request
        response = await call_next(request)

        # Add rate limit headers to response
        for key, value in headers.items():
            response.headers[key] = value

        return response


def rate_limit(
    limit: int = 100,
    window: int = 60,
    block_duration: int = 300,
    key_func: Optional[Callable[[Request], str]] = None,
    storage: Optional[Dict[str, List[float]]] = None,
    redis_url: Optional[str] = None,
    error_message: str = "Rate limit exceeded",
    retry_after: bool = True,
    rate_limiter: Optional[RateLimiter] = None
):
    """
    Dependency for rate limiting individual endpoints.

    Args:
        limit: Maximum number of requests per window
        window: Time window in seconds
        block_duration: Duration to block IPs that exceed the rate limit
        key_func: Function to extract key from request
        storage: Storage for request timestamps
        redis_url: Redis connection URL for distributed rate limiting
        error_message: Error message for rate limited requests
        retry_after: Whether to include Retry-After header
        rate_limiter: Optional existing RateLimiter instance

    Returns:
        Dependency function
    """
    # Use provided rate limiter or create a new one
    if rate_limiter:
        limiter = rate_limiter
    else:
        limiter = RateLimiter(
            limit=limit,
            window=window,
            block_duration=block_duration,
            key_func=key_func,
            storage=storage,
            redis_url=redis_url
        )

    logger = get_secure_logger("rate_limit_dependency")

    def dependency(request: Request, response: Response):
        # Get client IP for logging
        client_ip = request.client.host if request.client else "unknown"

        # Check rate limit
        is_limited, remaining, reset = limiter(request)

        # Set rate limit headers
        response.headers["X-RateLimit-Limit"] = str(limit)
        response.headers["X-RateLimit-Remaining"] = str(remaining)
        response.headers["X-RateLimit-Reset"] = str(reset)

        # Add Retry-After header if enabled
        if retry_after and is_limited:
            retry_after_seconds = max(1, reset - int(time.time()))
            response.headers["Retry-After"] = str(retry_after_seconds)

        # Raise exception if rate limited
        if is_limited:
            # Log the rate limit event
            logger.warning(
                "Rate limit exceeded",
                {
                    "client": client_ip,
                    "path": request.url.path,
                    "method": request.method,
                    "reset": datetime.fromtimestamp(reset).isoformat()
                }
            )

            # Prepare headers for the exception
            headers = {
                "X-RateLimit-Limit": str(limit),
                "X-RateLimit-Remaining": "0",
                "X-RateLimit-Reset": str(reset)
            }

            # Add Retry-After header if enabled
            if retry_after:
                retry_after_seconds = max(1, reset - int(time.time()))
                headers["Retry-After"] = str(retry_after_seconds)

            # Raise HTTP exception
            from fastapi import HTTPException
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "message": error_message,
                    "retry_after": reset - int(time.time())
                },
                headers=headers
            )

    return Depends(dependency)
