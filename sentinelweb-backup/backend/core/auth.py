"""
SentinelWeb Backend - Authentication

This module handles authentication and authorization for the SentinelWeb backend.
It integrates with BuloCloudSentinel's JWT authentication system.
"""

from datetime import datetime, timedelta, timezone
from typing import Optional, List

from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
# Replacing jose with PyJWT for security (CVE-2024-33664, CVE-2024-33663)
# from jwt.exceptions import PyJWTError as JWTError, jwt
import jwt
from jwt.exceptions import PyJWTError as JWTError
from pydantic import BaseModel

from backend.core.config import settings

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl=f"{settings.API_V1_STR}/auth/login")

# User models
class UserRole(BaseModel):
    name: str
    permissions: List[str]

class User(BaseModel):
    username: str
    email: Optional[str] = None
    full_name: Optional[str] = None
    disabled: Optional[bool] = None
    roles: List[UserRole] = []

class TokenData(BaseModel):
    username: Optional[str] = None
    exp: Optional[datetime] = None
    roles: List[str] = []

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token with the provided data and expiration time.

    Args:
        data: Dictionary containing data to encode in the token
        expires_delta: Optional timedelta for token expiration

    Returns:
        JWT token string
    """
    to_encode = data.copy()
    expire = datetime.now(timezone.utc) + (expires_delta or timedelta(minutes=settings.JWT_EXPIRATION_MINUTES))
    to_encode.update({"exp": expire})
    encoded_jwt = from security.auth.unified_auth import create_access_token as _create_access_token
    return _create_access_token(subject=to_encode)
    return encoded_jwt

async def get_current_user(token: str = Depends(oauth2_scheme)) -> User:
    """
    Validate the JWT token and return the current user.

    Args:
        token: JWT token from the Authorization header

    Returns:
        User object

    Raises:
        HTTPException: If token is invalid or user not found
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    try:
        # Decode JWT token with full validation
        payload = jwt.decode(
            token,
            settings.JWT_SECRET,
            algorithms=[settings.JWT_ALGORITHM],
            options={
                "verify_signature": True,
                "verify_exp": True,
                "verify_iat": True,
                "require": ["sub"]
            }
        )

        # Extract username and expiration
        username: str = payload.get("sub")
        if username is None:
            raise credentials_exception

        # Extract roles
        roles = payload.get("roles", [])

        # Create token data
        token_data = TokenData(username=username, roles=roles)

        # Check token expiration
        if token_data.exp and token_data.exp < datetime.now(timezone.utc):
            raise credentials_exception

    except JWTError:
        raise credentials_exception

    # Create user object
    user = User(
        username=token_data.username,
        roles=[UserRole(name=role, permissions=[]) for role in token_data.roles]
    )

    if user is None:
        raise credentials_exception

    return user

async def get_current_active_user(current_user: User = Depends(get_current_user)) -> User:
    """
    Check if the current user is active.

    Args:
        current_user: User object from get_current_user

    Returns:
        User object if active

    Raises:
        HTTPException: If user is disabled
    """
    if current_user.disabled:
        raise HTTPException(status_code=400, detail="Inactive user")
    return current_user

def has_role(required_roles: List[str]):
    """
    Dependency for checking if user has required roles.

    Args:
        required_roles: List of role names required for access

    Returns:
        Dependency function that checks user roles
    """
    async def check_roles(current_user: User = Depends(get_current_user)) -> User:
        user_roles = [role.name for role in current_user.roles]
        if not any(role in user_roles for role in required_roles):
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
        required_permission: Permission name required for access

    Returns:
        Dependency function that checks user permissions
    """
    async def check_permission(current_user: User = Depends(get_current_user)) -> User:
        for role in current_user.roles:
            if required_permission in role.permissions:
                return current_user

        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail=f"User does not have required permission: {required_permission}"
        )
    return check_permission
