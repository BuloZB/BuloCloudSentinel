"""
JWT Handler for Bulo.Cloud Sentinel Security Module.

This module provides secure JWT token generation, validation, and management.
"""

import os
import time
import logging
from datetime import datetime, timedelta, timezone
from typing import Dict, Optional, Any, Union

import jwt
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from pydantic import BaseModel

from .token_blacklist import blacklist_token, is_token_blacklisted

# Configure logging
logger = logging.getLogger(__name__)

# Configuration
JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "")
JWT_ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30
REFRESH_TOKEN_EXPIRE_DAYS = 7

if not JWT_SECRET_KEY:
    raise ValueError("JWT_SECRET_KEY environment variable must be set")

# OAuth2 scheme for token extraction
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

# Models
class TokenData(BaseModel):
    """Token data model."""
    sub: str
    exp: int
    iat: int
    jti: str
    type: str
    roles: list[str] = []
    permissions: list[str] = []
    additional_claims: Dict[str, Any] = {}

class Token(BaseModel):
    """Token response model."""
    access_token: str
    refresh_token: str
    token_type: str
    expires_in: int

def create_access_token(
    subject: Union[str, int],
    roles: list[str] = [],
    permissions: list[str] = [],
    additional_claims: Dict[str, Any] = {},
    expires_delta: Optional[timedelta] = None
) -> str:
    """
    Create a new JWT access token.

    Args:
        subject: The subject of the token (usually user ID)
        roles: List of user roles
        permissions: List of user permissions
        additional_claims: Additional claims to include in the token
        expires_delta: Optional custom expiration time

    Returns:
        JWT token string
    """
    if expires_delta is None:
        expires_delta = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)

    expire = datetime.now(timezone.utc) + expires_delta

    # Create token payload
    payload = {
        "sub": str(subject),
        "exp": expire.timestamp(),
        "iat": datetime.now(timezone.utc).timestamp(),
        "jti": os.urandom(16).hex(),  # Unique token ID
        "type": "access",
        "roles": roles,
        "permissions": permissions,
        **additional_claims
    }

    # Create token
    token = from security.auth.unified_auth import create_access_token as _create_access_token
    return _create_access_token(subject=payload)

    return token

def create_refresh_token(
    subject: Union[str, int],
    expires_delta: Optional[timedelta] = None
) -> str:
    """
    Create a new JWT refresh token.

    Args:
        subject: The subject of the token (usually user ID)
        expires_delta: Optional custom expiration time

    Returns:
        JWT token string
    """
    if expires_delta is None:
        expires_delta = timedelta(days=REFRESH_TOKEN_EXPIRE_DAYS)

    expire = datetime.now(timezone.utc) + expires_delta

    # Create token payload
    payload = {
        "sub": str(subject),
        "exp": expire.timestamp(),
        "iat": datetime.now(timezone.utc).timestamp(),
        "jti": os.urandom(16).hex(),  # Unique token ID
        "type": "refresh"
    }

    # Create token
    token = jwt.encode(payload, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)

    return token

def create_token_response(
    subject: Union[str, int],
    roles: list[str] = [],
    permissions: list[str] = [],
    additional_claims: Dict[str, Any] = {}
) -> Token:
    """
    Create a complete token response with access and refresh tokens.

    Args:
        subject: The subject of the token (usually user ID)
        roles: List of user roles
        permissions: List of user permissions
        additional_claims: Additional claims to include in the token

    Returns:
        Token response object
    """
    access_token = create_access_token(
        subject=subject,
        roles=roles,
        permissions=permissions,
        additional_claims=additional_claims
    )

    refresh_token = create_refresh_token(subject=subject)

    return Token(
        access_token=access_token,
        refresh_token=refresh_token,
        token_type="bearer",
        expires_in=ACCESS_TOKEN_EXPIRE_MINUTES * 60
    )

def decode_token(token: str) -> TokenData:
    """
    Decode and validate a JWT token.

    Args:
        token: JWT token string

    Returns:
        TokenData object

    Raises:
        HTTPException: If token is invalid or expired
    """
    try:
        # Decode token with full validation
        # options parameter ensures all required claims are present and validated
        payload = jwt.decode(
            token,
            JWT_SECRET_KEY,
            algorithms=[JWT_ALGORITHM],
            options={
                "verify_signature": True,
                "verify_exp": True,
                "verify_iat": True,
                "require": ["sub", "exp", "iat", "jti", "type"]
            }
        )

        # Validate required fields
        required_fields = ["sub", "exp", "iat", "jti", "type"]
        for field in required_fields:
            if field not in payload:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail=f"Token missing required field: {field}",
                    headers={"WWW-Authenticate": "Bearer"},
                )

        # Check if token is blacklisted
        if is_token_blacklisted(payload["jti"]):
            logger.warning(f"Blacklisted token used: {payload['jti']}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Token has been revoked",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Extract token data
        token_data = TokenData(
            sub=payload["sub"],
            exp=payload["exp"],
            iat=payload["iat"],
            jti=payload["jti"],
            type=payload["type"],
            roles=payload.get("roles", []),
            permissions=payload.get("permissions", []),
            additional_claims={k: v for k, v in payload.items() if k not in ["sub", "exp", "iat", "jti", "type", "roles", "permissions"]}
        )

        # Check if token is expired (redundant with verify_exp, but kept for clarity)
        if token_data.exp < time.time():
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Token has expired",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Check if token was issued in the future (clock skew)
        if token_data.iat > time.time() + 30:  # Allow 30 seconds of clock skew
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Token issued in the future",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Validate token type
        if token_data.type not in ["access", "refresh"]:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token type",
                headers={"WWW-Authenticate": "Bearer"},
            )

        return token_data

    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired",
            headers={"WWW-Authenticate": "Bearer"},
        )
    except jwt.InvalidTokenError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=f"Invalid token: {str(e)}",
            headers={"WWW-Authenticate": "Bearer"},
        )

async def get_current_token_data(token: str = Depends(oauth2_scheme)) -> TokenData:
    """
    Get current token data from request.

    Args:
        token: JWT token string from request

    Returns:
        TokenData object

    Raises:
        HTTPException: If token is invalid or expired
    """
    return decode_token(token)

async def get_current_user_id(token_data: TokenData = Depends(get_current_token_data)) -> str:
    """
    Get current user ID from token data.

    Args:
        token_data: Token data from request

    Returns:
        User ID string
    """
    return token_data.sub

def has_role(required_role: str):
    """
    Dependency for checking if user has a specific role.

    Args:
        required_role: Role to check for

    Returns:
        Dependency function
    """
    async def check_role(token_data: TokenData = Depends(get_current_token_data)):
        if required_role not in token_data.roles:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"Role '{required_role}' required",
            )
        return token_data

    return check_role

def has_permission(required_permission: str):
    """
    Dependency for checking if user has a specific permission.

    Args:
        required_permission: Permission to check for

    Returns:
        Dependency function
    """
    async def check_permission(token_data: TokenData = Depends(get_current_token_data)):
        if required_permission not in token_data.permissions:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"Permission '{required_permission}' required",
            )
        return token_data

    return check_permission


def logout(token_data: TokenData):
    """
    Logout a user by blacklisting their token.

    Args:
        token_data: Token data to blacklist
    """
    # Add token to blacklist
    blacklist_token(token_data.jti, token_data.exp)
    logger.info(f"User {token_data.sub} logged out, token {token_data.jti} blacklisted")


def logout_all_tokens(user_id: str) -> int:
    """
    Logout all tokens for a user.
    
    This function blacklists all active tokens for a user.
    
    Args:
        user_id: User ID to logout
        
    Returns:
        Number of tokens blacklisted
    """
    from .token_blacklist import blacklist_all_user_tokens
    
    # Blacklist all tokens for the user
    count = blacklist_all_user_tokens(user_id)
    
    logger.info(f"Logged out all tokens for user {user_id} ({count} tokens)")
    
    return count
