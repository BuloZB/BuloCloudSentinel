"""
Security utilities for the TACS module.
"""

import jwt
from fastapi import Depends, HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from datetime import datetime, timedelta, timezone
from typing import Dict, Any, Optional, List
import uuid
import logging

from core.config import settings
from db.session import get_db_session
from db.models import AuditLog

# Configure logging
logger = logging.getLogger(__name__)

# Security scheme
security = HTTPBearer()

async def create_access_token(data: Dict[str, Any], expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token.

    Args:
        data: Token data
        expires_delta: Optional expiration time

    Returns:
        JWT token
    """
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)

    to_encode.update({"exp": expire})

    encoded_jwt = from security.auth.unified_auth import create_access_token as _create_access_token
    return _create_access_token(subject=to_encode)

    return encoded_jwt

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    request: Request = None
) -> Dict[str, Any]:
    """
    Get the current authenticated user.

    Args:
        credentials: HTTP authorization credentials
        request: FastAPI request

    Returns:
        User data

    Raises:
        HTTPException: If authentication fails
    """
    try:
        # Get token
        token = credentials.credentials

        # Decode token with full validation
        payload = jwt.decode(
            token,
            settings.SECRET_KEY,
            algorithms=[settings.ALGORITHM],
            options={
                "verify_signature": True,
                "verify_exp": True,
                "verify_iat": True,
                "require": ["sub"]
            }
        )

        # Extract user data
        user_id = payload.get("sub")
        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid authentication credentials: missing subject claim",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Check if token was issued in the future (clock skew)
        if "iat" in payload and payload["iat"] > datetime.now(timezone.utc).timestamp() + 30:  # Allow 30 seconds of clock skew
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Token issued in the future",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Extract additional user data
        user_data = {
            "id": user_id,
            "username": payload.get("username", ""),
            "email": payload.get("email", ""),
            "roles": payload.get("roles", []),
            "permissions": payload.get("permissions", []),
            "clearance": payload.get("clearance", "unclassified")
        }

        return user_data

    except jwt.PyJWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

def has_permission(permission: str):
    """
    Check if the user has a specific permission.

    Args:
        permission: Required permission

    Returns:
        Dependency function
    """
    async def check_permission(user: Dict[str, Any] = Depends(get_current_user)) -> Dict[str, Any]:
        """
        Check if the user has the required permission.

        Args:
            user: User data

        Returns:
            User data

        Raises:
            HTTPException: If user doesn't have the required permission
        """
        # Check if user has the required permission
        if permission in user.get("permissions", []):
            return user

        # Check if user has admin role
        if "admin" in user.get("roles", []):
            return user

        # Permission denied
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail=f"Permission denied: {permission}",
        )

    return check_permission

def check_clearance(user: Dict[str, Any], required_clearance: str) -> bool:
    """
    Check if the user has the required clearance level.

    Args:
        user: User data
        required_clearance: Required clearance level

    Returns:
        True if user has the required clearance, False otherwise
    """
    # Clearance levels
    clearance_levels = {
        "unclassified": 0,
        "confidential": 1,
        "secret": 2,
        "top_secret": 3
    }

    # Get user clearance level
    user_clearance = user.get("clearance", "unclassified").lower()
    user_level = clearance_levels.get(user_clearance, 0)

    # Get required clearance level
    required_level = clearance_levels.get(required_clearance.lower(), 0)

    # Check if user has the required clearance
    return user_level >= required_level

async def log_security_event(
    event_type: str,
    user_id: str,
    resource_id: Optional[str] = None,
    resource_type: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    db = None
) -> None:
    """
    Log a security event.

    Args:
        event_type: Event type
        user_id: User ID
        resource_id: Optional resource ID
        resource_type: Optional resource type
        details: Optional event details
        ip_address: Optional IP address
        user_agent: Optional user agent
        db: Optional database session
    """
    try:
        # Create audit log
        audit_log = AuditLog(
            id=uuid.uuid4(),
            timestamp=datetime.now(timezone.utc),
            user_id=user_id,
            action=event_type,
            resource_type=resource_type or "",
            resource_id=resource_id,
            details=details or {},
            ip_address=ip_address,
            user_agent=user_agent
        )

        # Get database session if not provided
        if db is None:
            db = await get_db_session().__anext__()

        # Add to database
        db.add(audit_log)
        await db.commit()

    except Exception as e:
        logger.error(f"Failed to log security event: {e}")

def generate_api_key() -> str:
    """
    Generate a new API key.

    Returns:
        API key
    """
    return f"tacs_{uuid.uuid4().hex}"
