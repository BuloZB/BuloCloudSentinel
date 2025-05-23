"""
Security module for the Remote ID & Regulatory Compliance Service.

This module provides security-related functionality, including authentication,
authorization, and other security features.
"""

import time
from datetime import datetime, timedelta
from typing import Any, Dict, Optional, Union

import jwt
from fastapi import Depends, FastAPI, HTTPException, Security, status
from fastapi.security import OAuth2PasswordBearer, SecurityScopes
from passlib.context import CryptContext
from pydantic import BaseModel, ValidationError

from remoteid_service.core.settings import Settings, get_settings

# Password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# OAuth2 token URL
oauth2_scheme = OAuth2PasswordBearer(
    tokenUrl=f"{get_settings().API_V1_STR}/auth/token",
    scopes={
        "remoteid:read": "Read Remote ID data",
        "remoteid:write": "Write Remote ID data",
        "flightplan:read": "Read flight plans",
        "flightplan:write": "Submit flight plans",
        "notam:read": "Read NOTAM data",
    },
)

# Token models
class Token(BaseModel):
    """Token model."""
    access_token: str
    token_type: str
    expires_at: int

class TokenData(BaseModel):
    """Token data model."""
    username: Optional[str] = None
    scopes: list[str] = []
    exp: Optional[int] = None

# User model
class User(BaseModel):
    """User model."""
    username: str
    email: Optional[str] = None
    full_name: Optional[str] = None
    disabled: Optional[bool] = None
    scopes: list[str] = []

# Security functions
def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a password against a hash.
    
    Args:
        plain_password: Plain text password
        hashed_password: Hashed password
        
    Returns:
        bool: True if password matches hash
    """
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """
    Hash a password.
    
    Args:
        password: Plain text password
        
    Returns:
        str: Hashed password
    """
    return pwd_context.hash(password)

def create_access_token(
    data: Dict[str, Any],
    settings: Settings,
    expires_delta: Optional[timedelta] = None,
) -> str:
    """
    Create a JWT access token.
    
    Args:
        data: Token data
        settings: Application settings
        expires_delta: Token expiration time
        
    Returns:
        str: JWT token
    """
    to_encode = data.copy()
    
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.SECRET_KEY, algorithm=settings.ALGORITHM)
    
    return encoded_jwt

async def get_current_user(
    security_scopes: SecurityScopes,
    token: str = Depends(oauth2_scheme),
    settings: Settings = Depends(get_settings),
) -> User:
    """
    Get the current user from a JWT token.
    
    Args:
        security_scopes: Security scopes
        token: JWT token
        settings: Application settings
        
    Returns:
        User: Current user
        
    Raises:
        HTTPException: If authentication fails
    """
    if security_scopes.scopes:
        authenticate_value = f'Bearer scope="{security_scopes.scope_str}"'
    else:
        authenticate_value = "Bearer"
    
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": authenticate_value},
    )
    
    try:
        payload = jwt.decode(
            token, settings.SECRET_KEY, algorithms=[settings.ALGORITHM]
        )
        username: str = payload.get("sub")
        if username is None:
            raise credentials_exception
        
        token_scopes = payload.get("scopes", [])
        token_exp = payload.get("exp")
        
        token_data = TokenData(username=username, scopes=token_scopes, exp=token_exp)
    except (jwt.PyJWTError, ValidationError):
        raise credentials_exception
    
    # Check token expiration
    if token_data.exp is not None and token_data.exp < time.time():
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token expired",
            headers={"WWW-Authenticate": authenticate_value},
        )
    
    # In a real application, you would fetch the user from a database
    # For this example, we'll create a dummy user
    user = User(
        username=token_data.username,
        email=f"{token_data.username}@example.com",
        full_name=f"User {token_data.username}",
        disabled=False,
        scopes=token_data.scopes,
    )
    
    if user.disabled:
        raise HTTPException(status_code=400, detail="Inactive user")
    
    # Check if the user has the required scopes
    for scope in security_scopes.scopes:
        if scope not in token_data.scopes:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"Not enough permissions. Required: {scope}",
                headers={"WWW-Authenticate": authenticate_value},
            )
    
    return user

def setup_security(app: FastAPI) -> None:
    """
    Set up security for the FastAPI application.
    
    Args:
        app: FastAPI application
    """
    # Add security middleware, etc.
    pass
