"""
API endpoints for target management.
"""

from typing import List, Optional
from uuid import UUID
from fastapi import APIRouter, Depends, HTTPException, Query, status, Request
from sqlalchemy.ext.asyncio import AsyncSession
from datetime import datetime, timedelta

from api.schemas import (
    Target, TargetCreate, TargetUpdate,
    TargetType, TargetStatus, GeoLocation
)
from core.security import get_current_user, has_permission, log_security_event
from db.session import get_db_session
from services.target_tracker import TargetTracker

router = APIRouter()

@router.get("/", response_model=List[Target])
async def get_targets(
    skip: int = 0,
    limit: int = 100,
    target_type: Optional[TargetType] = None,
    status: Optional[TargetStatus] = None,
    min_confidence: Optional[float] = Query(None, ge=0.0, le=1.0),
    min_priority: Optional[int] = Query(None, ge=1, le=10),
    sensor_id: Optional[UUID] = None,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get all targets with optional filtering.
    
    Args:
        skip: Number of targets to skip
        limit: Maximum number of targets to return
        target_type: Filter by target type
        status: Filter by status
        min_confidence: Filter by minimum confidence
        min_priority: Filter by minimum priority
        sensor_id: Filter by sensor ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        List of targets
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Get targets
    targets = await target_tracker.get_targets(
        skip=skip,
        limit=limit,
        target_type=target_type.value if target_type else None,
        status=status.value if status else None,
        min_confidence=min_confidence,
        min_priority=min_priority,
        sensor_id=str(sensor_id) if sensor_id else None
    )
    
    return targets

@router.post("/", response_model=Target, status_code=status.HTTP_201_CREATED)
async def create_target(
    target: TargetCreate,
    current_user = Depends(has_permission("tacs:create_target")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Create a new target.
    
    Args:
        target: Target data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Created target
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Create target
    created_target = await target_tracker.create_target(target)
    
    # Log the operation
    await log_security_event(
        event_type="target_created",
        user_id=current_user.id,
        resource_id=str(created_target.id),
        resource_type="target",
        details={
            "name": target.name,
            "type": target.type.value,
            "confidence": target.confidence
        }
    )
    
    return created_target

@router.get("/{target_id}", response_model=Target)
async def get_target(
    target_id: UUID,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get a specific target by ID.
    
    Args:
        target_id: Target ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Target
        
    Raises:
        HTTPException: If target not found
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Get target
    target = await target_tracker.get_target(str(target_id))
    
    # Check if target exists
    if not target:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Target with ID {target_id} not found"
        )
    
    return target

@router.put("/{target_id}", response_model=Target)
async def update_target(
    target_id: UUID,
    target_update: TargetUpdate,
    current_user = Depends(has_permission("tacs:update_target")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Update a target.
    
    Args:
        target_id: Target ID
        target_update: Target update data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Updated target
        
    Raises:
        HTTPException: If target not found
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Update target
    updated_target = await target_tracker.update_target(str(target_id), target_update)
    
    # Check if target exists
    if not updated_target:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Target with ID {target_id} not found"
        )
    
    # Log the operation
    await log_security_event(
        event_type="target_updated",
        user_id=current_user.id,
        resource_id=str(target_id),
        resource_type="target",
        details={
            "updated_fields": [k for k, v in target_update.dict(exclude_unset=True).items() if v is not None]
        }
    )
    
    return updated_target

@router.delete("/{target_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_target(
    target_id: UUID,
    current_user = Depends(has_permission("tacs:delete_target")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Delete a target.
    
    Args:
        target_id: Target ID
        current_user: Current authenticated user with required permission
        db: Database session
        
    Raises:
        HTTPException: If target not found
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Delete target
    success = await target_tracker.delete_target(str(target_id))
    
    # Check if target exists
    if not success:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Target with ID {target_id} not found"
        )
    
    # Log the operation
    await log_security_event(
        event_type="target_deleted",
        user_id=current_user.id,
        resource_id=str(target_id),
        resource_type="target",
        details={}
    )
    
    return None

@router.post("/{target_id}/predict", response_model=GeoLocation)
async def predict_target_location(
    target_id: UUID,
    time_delta: float = Query(..., gt=0, description="Time delta in seconds"),
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Predict a target's location after a time delta.
    
    Args:
        target_id: Target ID
        time_delta: Time delta in seconds
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Predicted location
        
    Raises:
        HTTPException: If target not found
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Predict location
    predicted_location = await target_tracker.predict_target_location(str(target_id), time_delta)
    
    # Check if target exists
    if not predicted_location:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Target with ID {target_id} not found"
        )
    
    return predicted_location
