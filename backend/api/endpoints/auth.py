"""
SentinelWeb Backend - Authentication Endpoints

This module provides endpoints for user authentication and token management.
"""

import time
import uuid
from datetime import timedelta, datetime, timezone
from typing import Any

from fastapi import APIRouter, Depends, HTTPException, status, Request
from fastapi.security import OAuth2PasswordRequestForm
from sqlalchemy.ext.asyncio import AsyncSession

from backend.core.auth import create_access_token, get_current_user, User
from backend.core.config import settings
from backend.db.session import get_db_session
from backend.services.sentinel_client import SentinelClient
from backend.services.user_service import UserService

router = APIRouter()

# Simple rate limiting
failed_attempts = {}
MAX_ATTEMPTS = 5
LOCKOUT_TIME = 15 * 60  # 15 minutes in seconds

def check_rate_limit(request: Request, username: str) -> None:
    """
    Check if a request is rate limited.

    Args:
        request: The request object
        username: The username being authenticated

    Raises:
        HTTPException: If the request is rate limited
    """
    client_ip = request.client.host
    current_time = time.time()

    # Create a unique key for this IP + username combination
    key = f"{client_ip}:{username}"

    # Check if the IP is currently locked out
    if key in failed_attempts:
        attempts, lockout_time = failed_attempts[key]

        # If the lockout period has expired, reset the counter
        if current_time - lockout_time > LOCKOUT_TIME and attempts >= MAX_ATTEMPTS:
            failed_attempts[key] = (0, current_time)
        # If the IP is locked out, raise an exception
        elif attempts >= MAX_ATTEMPTS:
            # Calculate remaining lockout time
            remaining = int(LOCKOUT_TIME - (current_time - lockout_time))
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail=f"Too many failed login attempts. Please try again later.",
                headers={"Retry-After": str(remaining)}
            )

def record_failed_attempt(request: Request, username: str) -> None:
    """
    Record a failed login attempt.

    Args:
        request: The request object
        username: The username being authenticated
    """
    client_ip = request.client.host
    current_time = time.time()

    # Create a unique key for this IP + username combination
    key = f"{client_ip}:{username}"

    # Increment the failed attempt counter
    if key in failed_attempts:
        attempts, _ = failed_attempts[key]
        failed_attempts[key] = (attempts + 1, current_time)
    else:
        failed_attempts[key] = (1, current_time)

def reset_failed_attempts(request: Request, username: str) -> None:
    """
    Reset failed login attempts after a successful login.

    Args:
        request: The request object
        username: The username being authenticated
    """
    client_ip = request.client.host
    key = f"{client_ip}:{username}"

    if key in failed_attempts:
        del failed_attempts[key]

@router.post("/login")
async def login(
    request: Request,
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
    # Check rate limiting before processing the login
    check_rate_limit(request, form_data.username)

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
            # Record failed attempt and use a generic error message
            record_failed_attempt(request, form_data.username)
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid credentials",
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

        # Reset failed attempts on successful login
        reset_failed_attempts(request, form_data.username)

        # Return the token from BuloCloudSentinel
        return {
            "access_token": token_data["access_token"],
            "token_type": "bearer",
            "expires_in": settings.JWT_EXPIRATION_MINUTES * 60
        }

    except Exception as e:
        # If BuloCloudSentinel authentication fails, try local authentication
        user = await user_service.authenticate(
            username=form_data.username,
            password=form_data.password
        )

        if not user:
            # Record failed attempt and use a generic error message
            record_failed_attempt(request, form_data.username)
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Update last login
        await user_service.update_last_login(user.username)

        # Reset failed attempts on successful login
        reset_failed_attempts(request, form_data.username)

        # Create access token
        access_token_expires = timedelta(minutes=settings.JWT_EXPIRATION_MINUTES)
        access_token = create_access_token(
            data={
                "sub": user.username,
                "roles": [role.name for role in user.roles],
                "iat": datetime.now(timezone.utc).timestamp(),
                "jti": str(uuid.uuid4())  # Add unique token ID to prevent replay attacks
            },
            expires_delta=access_token_expires
        )

        return {
            "access_token": access_token,
            "token_type": "bearer",
            "expires_in": settings.JWT_EXPIRATION_MINUTES * 60
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
