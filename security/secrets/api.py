"""
Secrets API for Bulo.Cloud Sentinel.

This module provides API endpoints for managing secrets.
"""

import logging
from typing import Dict, List, Optional, Any
from fastapi import APIRouter, Depends, HTTPException, status, Security
from pydantic import BaseModel, Field

from ..auth.jwt_handler import has_role, has_permission
from .secrets_manager import get_secret, set_secret, delete_secret, list_secrets
from .rotation import (
    add_secret_for_rotation,
    remove_secret_from_rotation,
    rotate_secret,
    rotate_all_secrets,
    get_rotation_status,
    start_secret_rotation,
    stop_secret_rotation
)

# Configure logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter(
    prefix="/secrets",
    tags=["secrets"],
    dependencies=[Security(has_permission("manage:secrets"))]
)


# Models
class SecretCreate(BaseModel):
    """Model for creating a secret."""
    value: str = Field(..., description="Secret value")
    backend_index: int = Field(0, description="Backend index")


class SecretRotationConfig(BaseModel):
    """Model for configuring secret rotation."""
    interval: Optional[int] = Field(None, description="Rotation interval in seconds")
    backend_index: int = Field(0, description="Backend index")


class SecretRotationStatus(BaseModel):
    """Model for secret rotation status."""
    last_rotation: str = Field(..., description="Last rotation time")
    next_rotation: str = Field(..., description="Next rotation time")
    time_until_next: str = Field(..., description="Time until next rotation")
    percentage: float = Field(..., description="Percentage until next rotation")
    interval: str = Field(..., description="Rotation interval")
    due: bool = Field(..., description="Whether rotation is due")


# Endpoints
@router.get("/")
async def list_all_secrets(
    prefix: Optional[str] = None,
    backend_index: Optional[int] = None
) -> List[str]:
    """
    List all secrets.
    
    Args:
        prefix: Optional prefix to filter secrets
        backend_index: Backend index
        
    Returns:
        List of secret keys
    """
    try:
        return list_secrets(prefix, backend_index)
    except Exception as e:
        logger.error(f"Error listing secrets: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error listing secrets: {str(e)}"
        )


@router.get("/{key}")
async def get_secret_value(
    key: str,
    default: Optional[str] = None
) -> Dict[str, Any]:
    """
    Get a secret value.
    
    Args:
        key: Secret key
        default: Default value if secret is not found
        
    Returns:
        Secret value
    """
    try:
        value = get_secret(key, default)
        if value is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Secret '{key}' not found"
            )
        
        # Mask the value in the response
        masked_value = "*" * 8
        if value and len(value) > 4:
            masked_value = value[:2] + "*" * (len(value) - 4) + value[-2:]
        
        return {
            "key": key,
            "value": masked_value,
            "exists": True
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting secret '{key}': {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting secret: {str(e)}"
        )


@router.post("/{key}")
async def set_secret_value(
    key: str,
    secret: SecretCreate
) -> Dict[str, Any]:
    """
    Set a secret value.
    
    Args:
        key: Secret key
        secret: Secret data
        
    Returns:
        Success status
    """
    try:
        success = set_secret(key, secret.value, secret.backend_index)
        if not success:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Failed to set secret '{key}'"
            )
        
        return {
            "key": key,
            "success": True
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error setting secret '{key}': {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error setting secret: {str(e)}"
        )


@router.delete("/{key}")
async def delete_secret_value(
    key: str,
    backend_index: Optional[int] = None
) -> Dict[str, Any]:
    """
    Delete a secret.
    
    Args:
        key: Secret key
        backend_index: Backend index
        
    Returns:
        Success status
    """
    try:
        success = delete_secret(key, backend_index)
        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Secret '{key}' not found or could not be deleted"
            )
        
        return {
            "key": key,
            "success": True
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting secret '{key}': {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error deleting secret: {str(e)}"
        )


@router.post("/{key}/rotate")
async def rotate_secret_value(
    key: str
) -> Dict[str, Any]:
    """
    Rotate a secret.
    
    Args:
        key: Secret key
        
    Returns:
        Success status
    """
    try:
        success = rotate_secret(key)
        if not success:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Failed to rotate secret '{key}'"
            )
        
        return {
            "key": key,
            "success": True
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error rotating secret '{key}': {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error rotating secret: {str(e)}"
        )


@router.post("/{key}/rotation")
async def configure_secret_rotation(
    key: str,
    config: SecretRotationConfig
) -> Dict[str, Any]:
    """
    Configure secret rotation.
    
    Args:
        key: Secret key
        config: Rotation configuration
        
    Returns:
        Success status
    """
    try:
        # Check if secret exists
        value = get_secret(key)
        if value is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Secret '{key}' not found"
            )
        
        # Configure rotation
        add_secret_for_rotation(
            key=key,
            interval=config.interval,
            backend_index=config.backend_index
        )
        
        return {
            "key": key,
            "success": True,
            "message": "Secret rotation configured"
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error configuring rotation for secret '{key}': {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error configuring rotation: {str(e)}"
        )


@router.delete("/{key}/rotation")
async def disable_secret_rotation(
    key: str
) -> Dict[str, Any]:
    """
    Disable secret rotation.
    
    Args:
        key: Secret key
        
    Returns:
        Success status
    """
    try:
        remove_secret_from_rotation(key)
        
        return {
            "key": key,
            "success": True,
            "message": "Secret rotation disabled"
        }
    except Exception as e:
        logger.error(f"Error disabling rotation for secret '{key}': {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error disabling rotation: {str(e)}"
        )


@router.get("/rotation/status")
async def get_secret_rotation_status() -> Dict[str, SecretRotationStatus]:
    """
    Get secret rotation status.
    
    Returns:
        Dictionary mapping secret keys to rotation status
    """
    try:
        return get_rotation_status()
    except Exception as e:
        logger.error(f"Error getting rotation status: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting rotation status: {str(e)}"
        )


@router.post("/rotation/start")
async def start_automatic_rotation(
    check_interval: int = 60
) -> Dict[str, Any]:
    """
    Start automatic secret rotation.
    
    Args:
        check_interval: Interval in seconds to check for due secrets
        
    Returns:
        Success status
    """
    try:
        start_secret_rotation(check_interval)
        
        return {
            "success": True,
            "message": f"Automatic secret rotation started with check interval {check_interval}s"
        }
    except Exception as e:
        logger.error(f"Error starting automatic rotation: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error starting automatic rotation: {str(e)}"
        )


@router.post("/rotation/stop")
async def stop_automatic_rotation() -> Dict[str, Any]:
    """
    Stop automatic secret rotation.
    
    Returns:
        Success status
    """
    try:
        stop_secret_rotation()
        
        return {
            "success": True,
            "message": "Automatic secret rotation stopped"
        }
    except Exception as e:
        logger.error(f"Error stopping automatic rotation: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error stopping automatic rotation: {str(e)}"
        )


@router.post("/rotation/rotate-all")
async def rotate_all_secret_values() -> Dict[str, Any]:
    """
    Rotate all secrets.
    
    Returns:
        Dictionary mapping secret keys to rotation success
    """
    try:
        results = rotate_all_secrets()
        
        # Count successes and failures
        successes = sum(1 for success in results.values() if success)
        failures = sum(1 for success in results.values() if not success)
        
        return {
            "success": True,
            "results": results,
            "summary": {
                "total": len(results),
                "successes": successes,
                "failures": failures
            }
        }
    except Exception as e:
        logger.error(f"Error rotating all secrets: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error rotating all secrets: {str(e)}"
        )
