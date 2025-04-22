"""
API endpoints for EW platform management.
"""

from typing import List, Optional
from uuid import UUID
from fastapi import APIRouter, Depends, HTTPException, Query, status, Request
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete

from api.schemas import EwPlatform, EwPlatformCreate, EwPlatformUpdate
from core.security import get_current_user, has_permission, log_security_event
from db.models import EwPlatform as EwPlatformModel
from db.session import get_db_session

router = APIRouter()

@router.get("/", response_model=List[EwPlatform])
async def get_platforms(
    skip: int = 0,
    limit: int = 100,
    type: Optional[str] = None,
    capability: Optional[str] = None,
    status: Optional[str] = None,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get all EW platforms with optional filtering.
    
    Args:
        skip: Number of platforms to skip
        limit: Maximum number of platforms to return
        type: Filter by platform type
        capability: Filter by platform capability
        status: Filter by platform status
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        List of EW platforms
    """
    query = select(EwPlatformModel)
    
    # Apply filters
    if type:
        query = query.filter(EwPlatformModel.type == type)
    if capability:
        query = query.filter(EwPlatformModel.capabilities.contains([capability]))
    if status:
        query = query.filter(EwPlatformModel.status == status)
    
    # Apply pagination
    query = query.offset(skip).limit(limit)
    
    # Execute query
    result = await db.execute(query)
    platforms = result.scalars().all()
    
    return platforms

@router.post("/", response_model=EwPlatform, status_code=status.HTTP_201_CREATED)
async def create_platform(
    platform: EwPlatformCreate,
    request: Request,
    current_user = Depends(has_permission("ew:create")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Create a new EW platform.
    
    Args:
        platform: EW platform data
        request: FastAPI request
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Created EW platform
    """
    # Create platform model
    db_platform = EwPlatformModel(
        name=platform.name,
        type=platform.type,
        capabilities=platform.capabilities,
        frequency_range=platform.frequency_range.dict(),
        power_output=platform.power_output,
        status="offline",  # Default status
        location=platform.location.dict() if platform.location else None,
        orientation=platform.orientation.dict() if platform.orientation else None,
        configuration=platform.configuration,
        metadata=platform.metadata
    )
    
    # Add to database
    db.add(db_platform)
    await db.commit()
    await db.refresh(db_platform)
    
    # Register platform with platform manager
    await request.app.state.platform_manager.register_platform(
        str(db_platform.id),
        {
            "name": db_platform.name,
            "type": db_platform.type,
            "capabilities": db_platform.capabilities,
            "frequency_range": db_platform.frequency_range,
            "power_output": db_platform.power_output
        }
    )
    
    # Log security event
    await log_security_event(
        event_type="platform_created",
        user_id=current_user.id,
        resource_id=str(db_platform.id),
        resource_type="ew_platform",
        details={
            "name": platform.name,
            "type": platform.type,
            "capabilities": [str(cap) for cap in platform.capabilities]
        }
    )
    
    return db_platform

@router.get("/{platform_id}", response_model=EwPlatform)
async def get_platform(
    platform_id: UUID,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get a specific EW platform by ID.
    
    Args:
        platform_id: EW platform ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        EW platform
        
    Raises:
        HTTPException: If platform not found
    """
    # Get platform
    result = await db.execute(select(EwPlatformModel).filter(EwPlatformModel.id == platform_id))
    platform = result.scalars().first()
    
    # Check if platform exists
    if not platform:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"EW platform with ID {platform_id} not found"
        )
    
    return platform

@router.put("/{platform_id}", response_model=EwPlatform)
async def update_platform(
    platform_id: UUID,
    platform_update: EwPlatformUpdate,
    request: Request,
    current_user = Depends(has_permission("ew:update")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Update an EW platform.
    
    Args:
        platform_id: EW platform ID
        platform_update: EW platform update data
        request: FastAPI request
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Updated EW platform
        
    Raises:
        HTTPException: If platform not found
    """
    # Get platform
    result = await db.execute(select(EwPlatformModel).filter(EwPlatformModel.id == platform_id))
    db_platform = result.scalars().first()
    
    # Check if platform exists
    if not db_platform:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"EW platform with ID {platform_id} not found"
        )
    
    # Update platform fields
    update_data = platform_update.dict(exclude_unset=True)
    
    # Handle nested objects
    if "location" in update_data and update_data["location"]:
        update_data["location"] = update_data["location"].dict()
    if "orientation" in update_data and update_data["orientation"]:
        update_data["orientation"] = update_data["orientation"].dict()
    if "frequency_range" in update_data and update_data["frequency_range"]:
        update_data["frequency_range"] = update_data["frequency_range"].dict()
    
    # Update platform
    for key, value in update_data.items():
        setattr(db_platform, key, value)
    
    # Commit changes
    await db.commit()
    await db.refresh(db_platform)
    
    # Update platform in platform manager
    if "configuration" in update_data or "status" in update_data:
        await request.app.state.platform_manager.update_platform_config(
            str(db_platform.id),
            {
                "configuration": db_platform.configuration,
                "status": db_platform.status
            }
        )
    
    # Log security event
    await log_security_event(
        event_type="platform_updated",
        user_id=current_user.id,
        resource_id=str(db_platform.id),
        resource_type="ew_platform",
        details={
            "updated_fields": list(update_data.keys())
        }
    )
    
    return db_platform

@router.delete("/{platform_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_platform(
    platform_id: UUID,
    request: Request,
    current_user = Depends(has_permission("ew:delete")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Delete an EW platform.
    
    Args:
        platform_id: EW platform ID
        request: FastAPI request
        current_user: Current authenticated user with required permission
        db: Database session
        
    Raises:
        HTTPException: If platform not found
    """
    # Get platform
    result = await db.execute(select(EwPlatformModel).filter(EwPlatformModel.id == platform_id))
    db_platform = result.scalars().first()
    
    # Check if platform exists
    if not db_platform:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"EW platform with ID {platform_id} not found"
        )
    
    # Unregister platform from platform manager
    await request.app.state.platform_manager.unregister_platform(str(platform_id))
    
    # Delete platform
    await db.delete(db_platform)
    await db.commit()
    
    # Log security event
    await log_security_event(
        event_type="platform_deleted",
        user_id=current_user.id,
        resource_id=str(platform_id),
        resource_type="ew_platform",
        details={
            "name": db_platform.name,
            "type": db_platform.type
        }
    )
    
    return None
