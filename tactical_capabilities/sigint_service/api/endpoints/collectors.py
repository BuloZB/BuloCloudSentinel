"""
API endpoints for signal collector management.
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
from sqlalchemy.future import select
from sqlalchemy import update, delete

from api.schemas import SignalCollector, SignalCollectorCreate, SignalCollectorUpdate
from core.security import get_current_user, has_permission
from db.models import SignalCollector as SignalCollectorModel
from db.session import get_db_session

router = APIRouter()

@router.get("/", response_model=List[SignalCollector])
async def get_collectors(
    skip: int = 0,
    limit: int = 100,
    type: Optional[str] = None,
    platform_id: Optional[UUID] = None,
    status: Optional[str] = None,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get all signal collectors with optional filtering.
    
    Args:
        skip: Number of collectors to skip
        limit: Maximum number of collectors to return
        type: Filter by collector type
        platform_id: Filter by platform ID
        status: Filter by collector status
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        List of signal collectors
    """
    query = select(SignalCollectorModel)
    
    # Apply filters
    if type:
        query = query.filter(SignalCollectorModel.type == type)
    if platform_id:
        query = query.filter(SignalCollectorModel.platform_id == platform_id)
    if status:
        query = query.filter(SignalCollectorModel.status == status)
    
    # Apply pagination
    query = query.offset(skip).limit(limit)
    
    # Execute query
    result = await db.execute(query)
    collectors = result.scalars().all()
    
    return collectors

@router.post("/", response_model=SignalCollector, status_code=status.HTTP_201_CREATED)
async def create_collector(
    collector: SignalCollectorCreate,
    request: Request,
    current_user = Depends(has_permission("sigint:create")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Create a new signal collector.
    
    Args:
        collector: Signal collector data
        request: FastAPI request
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Created signal collector
    """
    # Create collector model
    db_collector = SignalCollectorModel(
        name=collector.name,
        type=collector.type,
        platform_id=collector.platform_id,
        status="offline",  # Default status
        location=collector.location.dict() if collector.location else None,
        orientation=collector.orientation.dict() if collector.orientation else None,
        frequency_range=collector.frequency_range.dict(),
        capabilities=collector.capabilities,
        configuration=collector.configuration,
        metadata=collector.metadata
    )
    
    # Add to database
    db.add(db_collector)
    await db.commit()
    await db.refresh(db_collector)
    
    # Register collector with collector manager
    await request.app.state.collector_manager.register_collector(
        str(db_collector.id),
        {
            "name": db_collector.name,
            "type": db_collector.type,
            "frequency_range": db_collector.frequency_range,
            "capabilities": db_collector.capabilities,
            "configuration": db_collector.configuration
        }
    )
    
    return db_collector

@router.get("/{collector_id}", response_model=SignalCollector)
async def get_collector(
    collector_id: UUID,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get a specific signal collector by ID.
    
    Args:
        collector_id: Signal collector ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Signal collector
        
    Raises:
        HTTPException: If collector not found
    """
    # Get collector
    result = await db.execute(select(SignalCollectorModel).filter(SignalCollectorModel.id == collector_id))
    collector = result.scalars().first()
    
    # Check if collector exists
    if not collector:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Signal collector with ID {collector_id} not found"
        )
    
    return collector

@router.put("/{collector_id}", response_model=SignalCollector)
async def update_collector(
    collector_id: UUID,
    collector_update: SignalCollectorUpdate,
    request: Request,
    current_user = Depends(has_permission("sigint:update")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Update a signal collector.
    
    Args:
        collector_id: Signal collector ID
        collector_update: Signal collector update data
        request: FastAPI request
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Updated signal collector
        
    Raises:
        HTTPException: If collector not found
    """
    # Get collector
    result = await db.execute(select(SignalCollectorModel).filter(SignalCollectorModel.id == collector_id))
    db_collector = result.scalars().first()
    
    # Check if collector exists
    if not db_collector:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Signal collector with ID {collector_id} not found"
        )
    
    # Update collector fields
    update_data = collector_update.dict(exclude_unset=True)
    
    # Handle nested objects
    if "location" in update_data and update_data["location"]:
        update_data["location"] = update_data["location"].dict()
    if "orientation" in update_data and update_data["orientation"]:
        update_data["orientation"] = update_data["orientation"].dict()
    if "frequency_range" in update_data and update_data["frequency_range"]:
        update_data["frequency_range"] = update_data["frequency_range"].dict()
    
    # Update collector
    for key, value in update_data.items():
        setattr(db_collector, key, value)
    
    # Commit changes
    await db.commit()
    await db.refresh(db_collector)
    
    # Update collector in collector manager
    if "configuration" in update_data or "status" in update_data:
        await request.app.state.collector_manager.update_collector_config(
            str(db_collector.id),
            {
                "configuration": db_collector.configuration,
                "status": db_collector.status
            }
        )
    
    return db_collector

@router.delete("/{collector_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_collector(
    collector_id: UUID,
    request: Request,
    current_user = Depends(has_permission("sigint:delete")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Delete a signal collector.
    
    Args:
        collector_id: Signal collector ID
        request: FastAPI request
        current_user: Current authenticated user with required permission
        db: Database session
        
    Raises:
        HTTPException: If collector not found
    """
    # Get collector
    result = await db.execute(select(SignalCollectorModel).filter(SignalCollectorModel.id == collector_id))
    db_collector = result.scalars().first()
    
    # Check if collector exists
    if not db_collector:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Signal collector with ID {collector_id} not found"
        )
    
    # Unregister collector from collector manager
    await request.app.state.collector_manager.unregister_collector(str(collector_id))
    
    # Delete collector
    await db.delete(db_collector)
    await db.commit()
    
    return None
