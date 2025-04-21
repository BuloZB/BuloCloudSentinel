"""
SentinelWeb Backend - Authentication Endpoints

This module provides endpoints for user authentication and token management.
"""

from datetime import timedelta
from typing import Any

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordRequestForm
from sqlalchemy.ext.asyncio import AsyncSession

from backend.core.auth import create_access_token, get_current_user, User
from backend.core.config import settings
from backend.db.session import get_db_session
from backend.services.sentinel_client import SentinelClient
from backend.services.user_service import UserService

router = APIRouter()

@router.post("/login")
async def login(
    form_data: OAuth2PasswordRequestForm = Depends(),
    db: AsyncSession = Depends(get_db_session)
) -> Any:
    """
    OAuth2 compatible token login, get an access token for future requests.
    
    Args:
        form_data: OAuth2 form with username and password
        db: Database session
        
    Returns:
        Dict with access token and token type
    """
    # Create user service
    user_service = UserService(db)
    
    # Try to authenticate with BuloCloudSentinel
    try:
        # Create sentinel client
        sentinel_client = SentinelClient(
            base_url=settings.SENTINEL_API_URL,
            token=None  # No token yet
        )
        
        # Authenticate with BuloCloudSentinel
        token_data = await sentinel_client.authenticate(
            username=form_data.username,
            password=form_data.password
        )
        
        if not token_data or not token_data.get("access_token"):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect username or password",
                headers={"WWW-Authenticate": "Bearer"},
            )
            
        # Get user info from token
        sentinel_client.token = token_data["access_token"]
        user_info = await sentinel_client.get_current_user()
        
        # Create or update user in local database
        user = await user_service.get_or_create_user(
            username=user_info["username"],
            email=user_info.get("email", f"{user_info['username']}@example.com"),
            roles=user_info.get("roles", ["user"])
        )
        
        # Update last login
        await user_service.update_last_login(user.username)
        
        # Return the token from BuloCloudSentinel
        return {
            "access_token": token_data["access_token"],
            "token_type": "bearer"
        }
        
    except Exception as e:
        # If BuloCloudSentinel authentication fails, try local authentication
        user = await user_service.authenticate(
            username=form_data.username,
            password=form_data.password
        )
        
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect username or password",
                headers={"WWW-Authenticate": "Bearer"},
            )
            
        # Update last login
        await user_service.update_last_login(user.username)
        
        # Create access token
        access_token_expires = timedelta(minutes=settings.JWT_EXPIRATION_MINUTES)
        access_token = create_access_token(
            data={"sub": user.username, "roles": [role.name for role in user.roles]},
            expires_delta=access_token_expires
        )
        
        return {
            "access_token": access_token,
            "token_type": "bearer"
        }

@router.post("/refresh-token")
async def refresh_token(current_user: User = Depends(get_current_user)) -> Any:
    """
    Refresh access token.
    
    Args:
        current_user: Current authenticated user
        
    Returns:
        Dict with new access token and token type
    """
    # Create new access token
    access_token_expires = timedelta(minutes=settings.JWT_EXPIRATION_MINUTES)
    access_token = create_access_token(
        data={"sub": current_user.username, "roles": [role.name for role in current_user.roles]},
        expires_delta=access_token_expires
    )
    
    return {
        "access_token": access_token,
        "token_type": "bearer"
    }

@router.get("/me")
async def get_me(current_user: User = Depends(get_current_user)) -> Any:
    """
    Get current user information.
    
    Args:
        current_user: Current authenticated user
        
    Returns:
        Current user information
    """
    return current_user
