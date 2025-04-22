"""
API endpoints for sensor management.
"""

from typing import List, Optional
from uuid import UUID
from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete

from api.schemas import Sensor, SensorCreate, SensorUpdate
from core.security import get_current_user, has_permission
from db.models import Sensor as SensorModel
from db.session import get_db_session

router = APIRouter()

@router.get("/", response_model=List[Sensor])
async def get_sensors(
    skip: int = 0,
    limit: int = 100,
    type: Optional[str] = None,
    platform_id: Optional[UUID] = None,
    status: Optional[str] = None,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get all sensors with optional filtering.
    
    Args:
        skip: Number of sensors to skip
        limit: Maximum number of sensors to return
        type: Filter by sensor type
        platform_id: Filter by platform ID
        status: Filter by sensor status
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        List of sensors
    """
    query = select(SensorModel)
    
    # Apply filters
    if type:
        query = query.filter(SensorModel.type == type)
    if platform_id:
        query = query.filter(SensorModel.platform_id == platform_id)
    if status:
        query = query.filter(SensorModel.status == status)
    
    # Apply pagination
    query = query.offset(skip).limit(limit)
    
    # Execute query
    result = await db.execute(query)
    sensors = result.scalars().all()
    
    return sensors

@router.post("/", response_model=Sensor, status_code=status.HTTP_201_CREATED)
async def create_sensor(
    sensor: SensorCreate,
    current_user = Depends(has_permission("sensors:create")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Create a new sensor.
    
    Args:
        sensor: Sensor data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Created sensor
    """
    # Create sensor model
    db_sensor = SensorModel(
        name=sensor.name,
        type=sensor.type,
        platform_id=sensor.platform_id,
        status="offline",  # Default status
        location=sensor.location.dict() if sensor.location else None,
        orientation=sensor.orientation.dict() if sensor.orientation else None,
        capabilities=sensor.capabilities,
        configuration=sensor.configuration,
        metadata=sensor.metadata
    )
    
    # Add to database
    db.add(db_sensor)
    await db.commit()
    await db.refresh(db_sensor)
    
    return db_sensor

@router.get("/{sensor_id}", response_model=Sensor)
async def get_sensor(
    sensor_id: UUID,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get a specific sensor by ID.
    
    Args:
        sensor_id: Sensor ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Sensor
        
    Raises:
        HTTPException: If sensor not found
    """
    # Get sensor
    result = await db.execute(select(SensorModel).filter(SensorModel.id == sensor_id))
    sensor = result.scalars().first()
    
    # Check if sensor exists
    if not sensor:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Sensor with ID {sensor_id} not found"
        )
    
    return sensor

@router.put("/{sensor_id}", response_model=Sensor)
async def update_sensor(
    sensor_id: UUID,
    sensor_update: SensorUpdate,
    current_user = Depends(has_permission("sensors:update")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Update a sensor.
    
    Args:
        sensor_id: Sensor ID
        sensor_update: Sensor update data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Updated sensor
        
    Raises:
        HTTPException: If sensor not found
    """
    # Get sensor
    result = await db.execute(select(SensorModel).filter(SensorModel.id == sensor_id))
    db_sensor = result.scalars().first()
    
    # Check if sensor exists
    if not db_sensor:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Sensor with ID {sensor_id} not found"
        )
    
    # Update sensor fields
    update_data = sensor_update.dict(exclude_unset=True)
    
    # Handle nested objects
    if "location" in update_data and update_data["location"]:
        update_data["location"] = update_data["location"].dict()
    if "orientation" in update_data and update_data["orientation"]:
        update_data["orientation"] = update_data["orientation"].dict()
    
    # Update sensor
    for key, value in update_data.items():
        setattr(db_sensor, key, value)
    
    # Commit changes
    await db.commit()
    await db.refresh(db_sensor)
    
    return db_sensor

@router.delete("/{sensor_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_sensor(
    sensor_id: UUID,
    current_user = Depends(has_permission("sensors:delete")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Delete a sensor.
    
    Args:
        sensor_id: Sensor ID
        current_user: Current authenticated user with required permission
        db: Database session
        
    Raises:
        HTTPException: If sensor not found
    """
    # Get sensor
    result = await db.execute(select(SensorModel).filter(SensorModel.id == sensor_id))
    db_sensor = result.scalars().first()
    
    # Check if sensor exists
    if not db_sensor:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Sensor with ID {sensor_id} not found"
        )
    
    # Delete sensor
    await db.delete(db_sensor)
    await db.commit()
    
    return None
