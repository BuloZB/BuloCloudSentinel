"""
API endpoints for track management.
"""

from typing import List, Optional

from security.validation.unified_validation import (
    validate_email,
    validate_username,
    validate_name,
    validate_uuid,
    validate_url,
    sanitize_string,
    sanitize_html,
    check_sql_injection,
    input_validator,
    form_validator,
    request_validator,
)
from uuid import UUID
from fastapi import APIRouter, Depends, HTTPException, Query, status, Request
from sqlalchemy.ext.asyncio import AsyncSession
from datetime import datetime, timedelta

from api.schemas import (
    Track, TrackCreate, TrackUpdate,
    TrackPoint, TrackStatus
)
from core.security import get_current_user, has_permission, log_security_event
from db.session import get_db_session
from services.target_tracker import TargetTracker

router = APIRouter()

@router.get("/", response_model=List[Track])
async def get_tracks(
    skip: int = 0,
    limit: int = 100,
    target_id: Optional[UUID] = None,
    status: Optional[TrackStatus] = None,
    min_quality: Optional[float] = Query(None, ge=0.0, le=1.0),
    start_time_after: Optional[datetime] = None,
    end_time_before: Optional[datetime] = None,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get all tracks with optional filtering.
    
    Args:
        skip: Number of tracks to skip
        limit: Maximum number of tracks to return
        target_id: Filter by target ID
        status: Filter by status
        min_quality: Filter by minimum quality
        start_time_after: Filter by start time after
        end_time_before: Filter by end time before
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        List of tracks
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Get tracks
    tracks = await target_tracker.get_tracks(
        skip=skip,
        limit=limit,
        target_id=str(target_id) if target_id else None,
        status=status.value if status else None,
        min_quality=min_quality,
        start_time_after=start_time_after,
        end_time_before=end_time_before
    )
    
    return tracks

@router.post("/", response_model=Track, status_code=status.HTTP_201_CREATED)
async def create_track(
    track: TrackCreate,
    current_user = Depends(has_permission("tacs:create_track")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Create a new track.
    
    Args:
        track: Track data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Created track
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Create track
    created_track = await target_tracker.create_track(track)
    
    # Log the operation
    await log_security_event(
        event_type="track_created",
        user_id=current_user.id,
        resource_id=str(created_track.id),
        resource_type="track",
        details={
            "target_id": str(track.target_id),
            "quality": track.quality,
            "num_points": len(track.points)
        }
    )
    
    return created_track

@router.get("/{track_id}", response_model=Track)
async def get_track(
    track_id: UUID,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get a specific track by ID.
    
    Args:
        track_id: Track ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Track
        
    Raises:
        HTTPException: If track not found
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Get track
    track = await target_tracker.get_track(str(track_id))
    
    # Check if track exists
    if not track:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Track with ID {track_id} not found"
        )
    
    return track

@router.put("/{track_id}", response_model=Track)
async def update_track(
    track_id: UUID,
    track_update: TrackUpdate,
    current_user = Depends(has_permission("tacs:update_track")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Update a track.
    
    Args:
        track_id: Track ID
        track_update: Track update data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Updated track
        
    Raises:
        HTTPException: If track not found
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Update track
    updated_track = await target_tracker.update_track(str(track_id), track_update)
    
    # Check if track exists
    if not updated_track:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Track with ID {track_id} not found"
        )
    
    # Log the operation
    await log_security_event(
        event_type="track_updated",
        user_id=current_user.id,
        resource_id=str(track_id),
        resource_type="track",
        details={
            "updated_fields": [k for k, v in track_update.dict(exclude_unset=True).items() if v is not None]
        }
    )
    
    return updated_track

@router.delete("/{track_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_track(
    track_id: UUID,
    current_user = Depends(has_permission("tacs:delete_track")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Delete a track.
    
    Args:
        track_id: Track ID
        current_user: Current authenticated user with required permission
        db: Database session
        
    Raises:
        HTTPException: If track not found
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Delete track
    success = await target_tracker.delete_track(str(track_id))
    
    # Check if track exists
    if not success:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Track with ID {track_id} not found"
        )
    
    # Log the operation
    await log_security_event(
        event_type="track_deleted",
        user_id=current_user.id,
        resource_id=str(track_id),
        resource_type="track",
        details={}
    )
    
    return None

@router.post("/{track_id}/points", response_model=TrackPoint)
async def add_track_point(
    track_id: UUID,
    track_point: TrackPoint,
    current_user = Depends(has_permission("tacs:update_track")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Add a point to a track.
    
    Args:
        track_id: Track ID
        track_point: Track point data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Added track point
        
    Raises:
        HTTPException: If track not found
    """
    # Create target tracker
    target_tracker = TargetTracker(db)
    
    # Add track point
    added_point = await target_tracker.add_track_point(
        track_id=str(track_id),
        timestamp=track_point.timestamp,
        location=track_point.location,
        sensor_id=str(track_point.sensor_id),
        altitude=track_point.altitude,
        velocity=track_point.velocity,
        acceleration=track_point.acceleration,
        heading=track_point.heading,
        confidence=track_point.confidence
    )
    
    # Check if track exists
    if not added_point:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Track with ID {track_id} not found"
        )
    
    # Log the operation
    await log_security_event(
        event_type="track_point_added",
        user_id=current_user.id,
        resource_id=str(track_id),
        resource_type="track",
        details={
            "timestamp": track_point.timestamp.isoformat(),
            "sensor_id": str(track_point.sensor_id),
            "confidence": track_point.confidence
        }
    )
    
    return added_point
