"""
Security utilities for the ISR service.

This module provides JWT token handling, password hashing, and authentication dependencies.
"""

from datetime import datetime, timedelta, timezone
from typing import Any, Dict, List, Optional, Union

import jwt
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from jwt.exceptions import PyJWTError
from passlib.context import CryptContext
from pydantic import BaseModel

from core.config import settings

# Password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl=f"{settings.API_V1_STR}/auth/login")

# Token data model
class TokenData(BaseModel):
    """Token data model."""
    sub: str
    exp: Optional[datetime] = None
    roles: List[str] = []
    permissions: List[str] = []

# User model
class User(BaseModel):
    """User model."""
    id: str
    username: str
    email: Optional[str] = None
    roles: List[str] = []
    permissions: List[str] = []

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a password against a hash.

    Args:
        plain_password: Plain text password
        hashed_password: Hashed password

    Returns:
        True if password matches hash, False otherwise
    """
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """
    Hash a password.

    Args:
        password: Plain text password

    Returns:
        Hashed password
    """
    return pwd_context.hash(password)

def create_access_token(
    subject: Union[str, int],
    roles: List[str] = [],
    permissions: List[str] = [],
    expires_delta: Optional[timedelta] = None
) -> str:
    """
    Create a JWT access token.

    Args:
        subject: Subject of the token (usually user ID)
        roles: List of user roles
        permissions: List of user permissions
        expires_delta: Optional expiration time delta

    Returns:
        JWT token string
    """
    if expires_delta is None:
        expires_delta = timedelta(minutes=settings.JWT_EXPIRATION_MINUTES)

    expire = datetime.now(timezone.utc) + expires_delta

    to_encode = {
        "sub": str(subject),
        "exp": expire,
        "roles": roles,
        "permissions": permissions
    }

    encoded_jwt = jwt.encode(
        to_encode,
        settings.JWT_SECRET,
        algorithm=settings.JWT_ALGORITHM
    )

    return encoded_jwt

def decode_token(token: str) -> TokenData:
    """
    Decode a JWT token.

    Args:
        token: JWT token string

    Returns:
        TokenData object

    Raises:
        HTTPException: If token is invalid
    """
    try:
        # Decode token with full validation
        payload = jwt.decode(
            token,
            settings.JWT_SECRET,
            algorithms=[settings.JWT_ALGORITHM],
            options={
                "verify_signature": True,
                "verify_exp": True,
                "verify_iat": True,
                "require": ["sub", "exp"]
            }
        )

        # Validate required fields
        required_fields = ["sub", "exp"]
        for field in required_fields:
            if field not in payload:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail=f"Token missing required field: {field}",
                    headers={"WWW-Authenticate": "Bearer"},
                )

        token_data = TokenData(
            sub=payload["sub"],
            exp=datetime.fromtimestamp(payload["exp"], tz=timezone.utc),
            roles=payload.get("roles", []),
            permissions=payload.get("permissions", [])
        )

        # Check if token is expired (redundant with verify_exp, but kept for clarity)
        if token_data.exp < datetime.now(timezone.utc):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Token expired",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Check if token was issued in the future (clock skew)
        if "iat" in payload:
            iat_datetime = datetime.fromtimestamp(payload["iat"], tz=timezone.utc)
            if iat_datetime > datetime.now(timezone.utc) + timedelta(seconds=30):  # Allow 30 seconds of clock skew
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Token issued in the future",
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

async def get_current_user(token: str = Depends(oauth2_scheme)) -> User:
    """
    Get the current user from a JWT token.

    Args:
        token: JWT token string

    Returns:
        User object

    Raises:
        HTTPException: If token is invalid or user not found
    """
    token_data = decode_token(token)

    # In a real application, you would look up the user in the database
    # For now, we'll just create a user object from the token data
    user = User(
        id=token_data.sub,
        username=token_data.sub,
        roles=token_data.roles,
        permissions=token_data.permissions
    )

    return user

def has_role(required_roles: List[str]):
    """
    Dependency for checking if user has required roles.

    Args:
        required_roles: List of required roles

    Returns:
        Dependency function
    """
    async def check_roles(current_user: User = Depends(get_current_user)) -> User:
        if not any(role in current_user.roles for role in required_roles):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"User does not have required roles: {', '.join(required_roles)}"
            )
        return current_user

    return check_roles

def has_permission(required_permission: str):
    """
    Dependency for checking if user has required permission.

    Args:
        required_permission: Required permission

    Returns:
        Dependency function
    """
    async def check_permission(current_user: User = Depends(get_current_user)) -> User:
        if required_permission not in current_user.permissions:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"User does not have required permission: {required_permission}"
            )
        return current_user

    return check_permission
