"""
Authentication endpoints for the Remote ID & Regulatory Compliance Service.

This module provides API endpoints for authentication and authorization.
"""

import time
from datetime import datetime, timedelta
from typing import Any, Dict, Optional

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from pydantic import BaseModel

from remoteid_service.core.security import (
    Token,
    User,
    create_access_token,
    get_current_user,
    get_password_hash,
    verify_password,
)
from remoteid_service.core.settings import Settings, get_settings

router = APIRouter()

# Mock user database for demonstration
# In a real application, you would use a database
USERS_DB = {
    "testuser": {
        "username": "testuser",
        "full_name": "Test User",
        "email": "testuser@example.com",
        "hashed_password": get_password_hash("testpassword"),
        "disabled": False,
        "scopes": ["remoteid:read", "remoteid:write", "flightplan:read", "flightplan:write", "notam:read"],
    }
}

class LoginRequest(BaseModel):
    """Login request model."""
    username: str
    password: str
    scopes: Optional[list[str]] = []

@router.post("/token", response_model=Token)
async def login_for_access_token(
    form_data: OAuth2PasswordRequestForm = Depends(),
    settings: Settings = Depends(get_settings),
) -> Dict[str, Any]:
    """
    OAuth2 compatible token login, get an access token for future requests.
    
    Args:
        form_data: OAuth2 password request form
        settings: Application settings
        
    Returns:
        dict: Access token
        
    Raises:
        HTTPException: If authentication fails
    """
    user = USERS_DB.get(form_data.username)
    if not user or not verify_password(form_data.password, user["hashed_password"]):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # Check if user is disabled
    if user["disabled"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Inactive user",
        )
    
    # Check requested scopes against user's scopes
    scopes = []
    for scope in form_data.scopes:
        if scope in user["scopes"]:
            scopes.append(scope)
    
    # Create access token
    access_token_expires = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    expires_at = int((datetime.utcnow() + access_token_expires).timestamp())
    
    access_token = create_access_token(
        data={"sub": user["username"], "scopes": scopes},
        settings=settings,
        expires_delta=access_token_expires,
    )
    
    return {
        "access_token": access_token,
        "token_type": "bearer",
        "expires_at": expires_at,
    }

@router.post("/login", response_model=Token)
async def login(
    login_request: LoginRequest,
    settings: Settings = Depends(get_settings),
) -> Dict[str, Any]:
    """
    Login endpoint for non-OAuth2 clients.
    
    Args:
        login_request: Login request
        settings: Application settings
        
    Returns:
        dict: Access token
        
    Raises:
        HTTPException: If authentication fails
    """
    user = USERS_DB.get(login_request.username)
    if not user or not verify_password(login_request.password, user["hashed_password"]):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
        )
    
    # Check if user is disabled
    if user["disabled"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Inactive user",
        )
    
    # Check requested scopes against user's scopes
    scopes = []
    for scope in login_request.scopes:
        if scope in user["scopes"]:
            scopes.append(scope)
    
    # Create access token
    access_token_expires = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    expires_at = int((datetime.utcnow() + access_token_expires).timestamp())
    
    access_token = create_access_token(
        data={"sub": user["username"], "scopes": scopes},
        settings=settings,
        expires_delta=access_token_expires,
    )
    
    return {
        "access_token": access_token,
        "token_type": "bearer",
        "expires_at": expires_at,
    }

@router.get("/me", response_model=User)
async def read_users_me(
    current_user: User = Security(get_current_user, scopes=[]),
) -> User:
    """
    Get current user.
    
    Args:
        current_user: Current user
        
    Returns:
        User: Current user
    """
    return current_user
