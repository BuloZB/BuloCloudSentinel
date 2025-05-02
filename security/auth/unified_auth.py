"""
Unified authentication module for Bulo.Cloud Sentinel.

This module provides a standardized approach to authentication across all components
of the Bulo.Cloud Sentinel platform, using secure algorithms and practices.
"""

import os
import logging
from datetime import datetime, timedelta, timezone
from typing import Dict, List, Optional, Union, Any

import jwt
from argon2 import PasswordHasher
from argon2.exceptions import VerifyMismatchError, InvalidHash
from fastapi import Depends, HTTPException, status, Request
from fastapi.security import OAuth2PasswordBearer
from pydantic import BaseModel

# Set up logging
logger = logging.getLogger(__name__)

# Initialize Argon2 password hasher with secure parameters
# These parameters are based on OWASP recommendations
ph = PasswordHasher(
    time_cost=3,  # Number of iterations
    memory_cost=65536,  # 64 MB
    parallelism=4,  # Number of parallel threads
    hash_len=32,  # Length of the hash in bytes
    salt_len=16,  # Length of the salt in bytes
)

# JWT settings
JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "")
JWT_ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))
REFRESH_TOKEN_EXPIRE_DAYS = int(os.getenv("REFRESH_TOKEN_EXPIRE_DAYS", "7"))

# Token model
class TokenData(BaseModel):
    """Token data model."""
    sub: str
    exp: Optional[datetime] = None
    roles: List[str] = []
    permissions: List[str] = []
    jti: str = ""
    type: str = "access"

# Password handling functions
def hash_password(password: str) -> str:
    """
    Hash a password using Argon2id.

    Args:
        password: Plain text password

    Returns:
        Hashed password
    """
    return ph.hash(password)

def verify_password(hashed_password: str, plain_password: str) -> bool:
    """
    Verify a password against a hash.

    Args:
        hashed_password: Hashed password
        plain_password: Plain text password to verify

    Returns:
        True if password matches, False otherwise
    """
    try:
        ph.verify(hashed_password, plain_password)

        # Check if the password hash needs to be rehashed
        if ph.check_needs_rehash(hashed_password):
            logger.info("Password hash needs rehashing due to parameter changes")

        return True
    except VerifyMismatchError:
        return False
    except InvalidHash as e:
        logger.error(f"Invalid password hash format: {str(e)}")
        return False

def check_password_needs_rehash(hashed_password: str) -> bool:
    """
    Check if a password hash needs to be rehashed.

    Args:
        hashed_password: Hashed password

    Returns:
        True if the hash needs to be rehashed, False otherwise
    """
    try:
        return ph.check_needs_rehash(hashed_password)
    except InvalidHash:
        return True

# JWT token functions
def create_access_token(
    subject: Union[str, int],
    roles: List[str] = [],
    permissions: List[str] = [],
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
    token = jwt.encode(payload, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)

    return token

def create_refresh_token(
    subject: Union[str, int],
    additional_claims: Dict[str, Any] = {},
    expires_delta: Optional[timedelta] = None
) -> str:
    """
    Create a new JWT refresh token.

    Args:
        subject: The subject of the token (usually user ID)
        additional_claims: Additional claims to include in the token
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
        "type": "refresh",
        **additional_claims
    }

    # Create token
    token = jwt.encode(payload, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)

    return token

def decode_token(token: str) -> Dict[str, Any]:
    """
    Decode and validate a JWT token.

    Args:
        token: JWT token string

    Returns:
        Decoded token payload

    Raises:
        HTTPException: If the token is invalid
    """
    try:
        # Decode token
        payload = jwt.decode(
            token,
            JWT_SECRET_KEY,
            algorithms=[JWT_ALGORITHM],
            options={"verify_signature": True}
        )

        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired",
            headers={"WWW-Authenticate": "Bearer"},
        )
    except jwt.InvalidTokenError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token",
            headers={"WWW-Authenticate": "Bearer"},
        )

def create_token_response(
    subject: Union[str, int],
    roles: List[str] = [],
    permissions: List[str] = [],
    additional_claims: Dict[str, Any] = {}
) -> Dict[str, str]:
    """
    Create a token response with access and refresh tokens.

    Args:
        subject: The subject of the token (usually user ID)
        roles: List of user roles
        permissions: List of user permissions
        additional_claims: Additional claims to include in the token

    Returns:
        Dictionary with access and refresh tokens
    """
    access_token = create_access_token(
        subject=subject,
        roles=roles,
        permissions=permissions,
        additional_claims=additional_claims
    )

    refresh_token = create_refresh_token(
        subject=subject,
        additional_claims=additional_claims
    )

    return {
        "access_token": access_token,
        "refresh_token": refresh_token,
        "token_type": "bearer"
    }

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="api/auth/login")

def get_current_token_data(token: str = Depends(oauth2_scheme)) -> TokenData:
    """
    Get the current token data from a JWT token.

    Args:
        token: JWT token string

    Returns:
        TokenData object

    Raises:
        HTTPException: If the token is invalid
    """
    payload = decode_token(token)

    # Check token type
    if payload.get("type") != "access":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token type",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Create token data
    token_data = TokenData(
        sub=payload.get("sub", ""),
        exp=datetime.fromtimestamp(payload.get("exp", 0), tz=timezone.utc),
        roles=payload.get("roles", []),
        permissions=payload.get("permissions", []),
        jti=payload.get("jti", ""),
        type=payload.get("type", "access")
    )

    return token_data

def get_current_user_id(token_data: TokenData = Depends(get_current_token_data)) -> str:
    """
    Get the current user ID from token data.

    Args:
        token_data: TokenData object

    Returns:
        User ID

    Raises:
        HTTPException: If the token is invalid
    """
    if not token_data.sub:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return token_data.sub

def has_role(required_role: str, token_data: TokenData = Depends(get_current_token_data)) -> bool:
    """
    Check if the current user has a specific role.

    Args:
        required_role: Required role
        token_data: TokenData object

    Returns:
        True if the user has the role, False otherwise
    """
    return required_role in token_data.roles

def has_permission(required_permission: str, token_data: TokenData = Depends(get_current_token_data)) -> bool:
    """
    Check if the current user has a specific permission.

    Args:
        required_permission: Required permission
        token_data: TokenData object

    Returns:
        True if the user has the permission, False otherwise
    """
    return required_permission in token_data.permissions

# Create a dependency for requiring specific roles
def require_role(required_role: str):
    """
    Create a dependency that requires a specific role.

    Args:
        required_role: Required role

    Returns:
        Dependency function
    """
    def dependency(token_data: TokenData = Depends(get_current_token_data)):
        if not has_role(required_role, token_data):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"Role '{required_role}' required",
                headers={"WWW-Authenticate": "Bearer"},
            )
        return token_data
    return dependency

# Create a dependency for requiring specific permissions
def require_permission(required_permission: str):
    """
    Create a dependency that requires a specific permission.

    Args:
        required_permission: Required permission

    Returns:
        Dependency function
    """
    def dependency(token_data: TokenData = Depends(get_current_token_data)):
        if not has_permission(required_permission, token_data):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"Permission '{required_permission}' required",
                headers={"WWW-Authenticate": "Bearer"},
            )
        return token_data
    return dependency
