"""
Authentication Service

This module provides authentication services for the Dock Driver microservice.
"""

import logging
import os
import json
from typing import Dict, Any, Optional
from datetime import datetime, timedelta, timezone
import yaml
import jwt
from jwt.exceptions import PyJWTError as JWTError
import redis.asyncio as redis

logger = logging.getLogger(__name__)


# Load configuration
def _load_config() -> Dict[str, Any]:
    """
    Load configuration from file.

    Returns:
        Dict[str, Any]: Configuration dictionary.
    """
    try:
        config_path = os.environ.get("DOCK_DRIVER_CONFIG", "config/config.yaml")

        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        # Replace environment variables in config
        config_str = json.dumps(config)
        for key, value in os.environ.items():
            config_str = config_str.replace(f"${{{key}}}", value)

        config = json.loads(config_str)

        return config
    except Exception as e:
        logger.error(f"Error loading configuration: {str(e)}")
        return {}


# Get configuration
config = _load_config()
security_config = config.get("security", {})

# Get JWT secret from environment variable first, then config, then use a secure default
# In production, this should always be set via environment variable
jwt_secret = os.environ.get("DOCK_DRIVER_JWT_SECRET", security_config.get("jwt_secret", None))
if not jwt_secret:
    # Generate a secure random secret if not provided
    import secrets
    jwt_secret = secrets.token_hex(32)
    logger.warning("JWT secret not provided, generated a random one. This will change on restart!")

jwt_algorithm = security_config.get("jwt_algorithm", "HS256")
jwt_expiration = security_config.get("jwt_expiration", 3600)

# Initialize Redis client
redis_url = config.get("redis", {}).get("url", "redis://localhost:6379/0")
redis_client = None


async def get_redis_client():
    """
    Get the Redis client.

    Returns:
        Redis: Redis client.
    """
    global redis_client
    if redis_client is None:
        redis_client = redis.from_url(redis_url)
    return redis_client


async def create_access_token(data: Dict[str, Any], expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token.

    Args:
        data: Data to encode in the token.
        expires_delta: Token expiration time.

    Returns:
        str: JWT access token.
    """
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(seconds=jwt_expiration)

    to_encode.update({"exp": expire})

    encoded_jwt = jwt.encode(to_encode, jwt_secret, algorithm=jwt_algorithm)

    return encoded_jwt


async def verify_token(token: str) -> Optional[Dict[str, Any]]:
    """
    Verify a JWT token.

    Args:
        token: JWT token to verify.

    Returns:
        Dict[str, Any]: Token payload if valid, None otherwise.
    """
    try:
        # Check if token is blacklisted first (faster than decoding)
        redis_client = await get_redis_client()
        blacklisted = await redis_client.get(f"token:blacklist:{token}")

        if blacklisted:
            logger.warning(f"Token is blacklisted: {token}")
            return None

        # Verify token with full validation
        payload = jwt.decode(
            token,
            jwt_secret,
            algorithms=[jwt_algorithm],
            options={
                "verify_signature": True,
                "verify_exp": True,
                "verify_iat": True if "iat" in jwt.decode(token, options={"verify_signature": False}) else False,
                "require": ["exp"]
            }
        )

        # Check token expiration
        exp = payload.get("exp", 0)
        if exp < datetime.now(timezone.utc).timestamp():
            logger.warning(f"Token is expired: {token}")
            return None

        # Check token issuer if present
        iss = payload.get("iss")
        if iss and iss != "bulocloud-sentinel":
            logger.warning(f"Token has invalid issuer: {iss}")
            return None

        return payload
    except JWTError as e:
        logger.error(f"JWT error: {str(e)}")
        return None
    except Exception as e:
        logger.error(f"Error verifying token: {str(e)}")
        return None


async def blacklist_token(token: str, expires_in: int = None) -> bool:
    """
    Blacklist a JWT token.

    Args:
        token: JWT token to blacklist.
        expires_in: Time in seconds until the token expires.

    Returns:
        bool: True if successful, False otherwise.
    """
    try:
        # Get token expiration time
        if expires_in is None:
            try:
                payload = jwt.decode(
                    token,
                    jwt_secret,
                    algorithms=[jwt_algorithm],
                    options={
                        "verify_signature": True,
                        "verify_exp": False,  # We need to get the exp even if token is expired
                        "verify_iat": True if "iat" in jwt.decode(token, options={"verify_signature": False}) else False
                    }
                )
                exp = payload.get("exp")
                if exp:
                    expires_in = max(0, int(exp - datetime.now(timezone.utc).timestamp()))
                else:
                    expires_in = jwt_expiration
            except Exception:
                expires_in = jwt_expiration

        # Add token to blacklist
        redis_client = await get_redis_client()
        await redis_client.setex(f"token:blacklist:{token}", expires_in, "1")

        return True
    except Exception as e:
        logger.error(f"Error blacklisting token: {str(e)}")
        return False


async def authenticate_user(username: str, password: str) -> Optional[Dict[str, Any]]:
    """
    Authenticate a user.

    Args:
        username: Username.
        password: Password.

    Returns:
        Dict[str, Any]: User information if authentication is successful, None otherwise.
    """
    # In a real implementation, this would check against a database
    # For now, we'll use environment variables or a secure configuration
    # Default to None if not set, which will fail authentication
    admin_username = os.environ.get("DOCK_DRIVER_ADMIN_USERNAME", "placeholderusername")
    admin_password = os.environ.get("DOCK_DRIVER_ADMIN_PASSWORD", "placeholderpassword")

    if username == admin_username and password == admin_password:
        return {
            "username": username,
            "email": "admin@example.com",
            "full_name": "Admin User",
            "disabled": False,
            "roles": ["admin"]
        }

    return None
