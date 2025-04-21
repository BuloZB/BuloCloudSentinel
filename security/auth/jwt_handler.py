"""
JWT Handler for Bulo.Cloud Sentinel Security Module.

This module provides secure JWT token generation, validation, and management.
"""

import os
import time
from datetime import datetime, timedelta
from typing import Dict, Optional, Any, Union

import jwt
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from pydantic import BaseModel

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
    
    expire = datetime.utcnow() + expires_delta
    
    # Create token payload
    payload = {
        "sub": str(subject),
        "exp": expire.timestamp(),
        "iat": datetime.utcnow().timestamp(),
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
    
    expire = datetime.utcnow() + expires_delta
    
    # Create token payload
    payload = {
        "sub": str(subject),
        "exp": expire.timestamp(),
        "iat": datetime.utcnow().timestamp(),
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
        # Decode token
        payload = jwt.decode(token, JWT_SECRET_KEY, algorithms=[JWT_ALGORITHM])
        
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
        
        # Check if token is expired
        if token_data.exp < time.time():
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Token has expired",
                headers={"WWW-Authenticate": "Bearer"},
            )
        
        return token_data
    
    except jwt.PyJWTError as e:
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
