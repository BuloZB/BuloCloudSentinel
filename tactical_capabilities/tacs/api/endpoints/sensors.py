"""
API endpoints for sensor management.
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
from fastapi import APIRouter, Depends, HTTPException, Query, status, Request, Body
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from datetime import datetime

from api.schemas import (
    Sensor, SensorCreate, SensorUpdate,
    SensorType, SensorStatus
)
from core.security import get_current_user, has_permission, log_security_event
from db.session import get_db_session
from db.models import Sensor as SensorModel

router = APIRouter()

@router.get("/", response_model=List[Sensor])
async def get_sensors(
    skip: int = 0,
    limit: int = 100,
    sensor_type: Optional[SensorType] = None,
    status: Optional[SensorStatus] = None,
    platform_id: Optional[UUID] = None,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get all sensors with optional filtering.
    
    Args:
        skip: Number of sensors to skip
        limit: Maximum number of sensors to return
        sensor_type: Filter by sensor type
        status: Filter by status
        platform_id: Filter by platform ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        List of sensors
    """
    # Build query
    query = select(SensorModel)
    
    # Apply filters
    if sensor_type:
        query = query.filter(SensorModel.type == sensor_type.value)
    if status:
        query = query.filter(SensorModel.status == status.value)
    if platform_id:
        query = query.filter(SensorModel.platform_id == platform_id)
    
    # Apply pagination
    query = query.offset(skip).limit(limit)
    
    # Execute query
    result = await db.execute(query)
    sensors = result.scalars().all()
    
    # Convert to schemas
    return [_convert_to_sensor_schema(sensor) for sensor in sensors]

@router.post("/", response_model=Sensor, status_code=status.HTTP_201_CREATED)
async def create_sensor(
    sensor: SensorCreate,
    current_user = Depends(has_permission("tacs:create_sensor")),
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
    # Create sensor
    db_sensor = SensorModel(
        name=sensor.name,
        type=sensor.type.value,
        platform_id=sensor.platform_id,
        capabilities=[cap.value for cap in sensor.capabilities],
        resolution=sensor.resolution.dict() if sensor.resolution else None,
        field_of_view=sensor.field_of_view.dict() if sensor.field_of_view else None,
        range=sensor.range,
        accuracy=sensor.accuracy,
        status=sensor.status.value,
        metadata=sensor.metadata
    )
    
    # Add to database
    db.add(db_sensor)
    await db.commit()
    await db.refresh(db_sensor)
    
    # Log the operation
    await log_security_event(
        event_type="sensor_created",
        user_id=current_user.id,
        resource_id=str(db_sensor.id),
        resource_type="sensor",
        details={
            "name": sensor.name,
            "type": sensor.type.value,
            "platform_id": str(sensor.platform_id)
        }
    )
    
    # Convert to schema
    return _convert_to_sensor_schema(db_sensor)

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
    
    # Convert to schema
    return _convert_to_sensor_schema(sensor)

@router.put("/{sensor_id}", response_model=Sensor)
async def update_sensor(
    sensor_id: UUID,
    sensor_update: SensorUpdate,
    current_user = Depends(has_permission("tacs:update_sensor")),
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
    sensor = result.scalars().first()
    
    # Check if sensor exists
    if not sensor:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Sensor with ID {sensor_id} not found"
        )
    
    # Update fields
    update_data = sensor_update.dict(exclude_unset=True)
    
    # Handle nested objects
    if "resolution" in update_data and update_data["resolution"]:
        update_data["resolution"] = update_data["resolution"].dict()
    if "field_of_view" in update_data and update_data["field_of_view"]:
        update_data["field_of_view"] = update_data["field_of_view"].dict()
    
    # Handle enums
    if "type" in update_data and update_data["type"]:
        update_data["type"] = update_data["type"].value
    if "status" in update_data and update_data["status"]:
        update_data["status"] = update_data["status"].value
    if "capabilities" in update_data and update_data["capabilities"]:
        update_data["capabilities"] = [cap.value for cap in update_data["capabilities"]]
    
    # Update sensor
    for key, value in update_data.items():
        setattr(sensor, key, value)
    
    # Commit changes
    await db.commit()
    await db.refresh(sensor)
    
    # Log the operation
    await log_security_event(
        event_type="sensor_updated",
        user_id=current_user.id,
        resource_id=str(sensor_id),
        resource_type="sensor",
        details={
            "updated_fields": [k for k, v in update_data.items() if v is not None]
        }
    )
    
    # Convert to schema
    return _convert_to_sensor_schema(sensor)

@router.delete("/{sensor_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_sensor(
    sensor_id: UUID,
    current_user = Depends(has_permission("tacs:delete_sensor")),
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
    sensor = result.scalars().first()
    
    # Check if sensor exists
    if not sensor:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Sensor with ID {sensor_id} not found"
        )
    
    # Delete sensor
    await db.delete(sensor)
    await db.commit()
    
    # Log the operation
    await log_security_event(
        event_type="sensor_deleted",
        user_id=current_user.id,
        resource_id=str(sensor_id),
        resource_type="sensor",
        details={}
    )
    
    return None

@router.post("/{sensor_id}/calibrate")
async def calibrate_sensor(
    sensor_id: UUID,
    parameters: Optional[dict] = Body(None),
    current_user = Depends(has_permission("tacs:calibrate_sensor")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Calibrate a sensor.
    
    Args:
        sensor_id: Sensor ID
        parameters: Calibration parameters
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Calibration result
        
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
    
    # Update sensor status
    sensor.status = "calibrating"
    await db.commit()
    
    # In a real implementation, this would trigger a calibration process
    # For now, we'll just simulate it
    
    # Update sensor status and metadata
    sensor.status = "online"
    if not sensor.metadata:
        sensor.metadata = {}
    
    sensor.metadata["last_calibration"] = datetime.utcnow().isoformat()
    sensor.metadata["calibration_parameters"] = parameters or {}
    
    # Commit changes
    await db.commit()
    
    # Log the operation
    await log_security_event(
        event_type="sensor_calibrated",
        user_id=current_user.id,
        resource_id=str(sensor_id),
        resource_type="sensor",
        details={
            "parameters": parameters
        }
    )
    
    # Return result
    return {
        "message": "Sensor calibrated successfully",
        "sensor_id": str(sensor_id),
        "status": sensor.status,
        "calibration_time": sensor.metadata["last_calibration"]
    }


def validate_request_data(request_data: dict, schema: dict) -> dict:
    """
    Validate request data against a schema.

    Args:
        request_data: Request data to validate
        schema: Validation schema

    Returns:
        Validated request data
    """
    return request_validator.validate_request(request_data, schema)

def _convert_to_sensor_schema(sensor: SensorModel) -> Sensor:
    """
    Convert a sensor database model to a schema.
    
    Args:
        sensor: Sensor database model
        
    Returns:
        Sensor schema
    """
    from api.schemas import (
        SensorType, SensorStatus, SensorCapability,
        Resolution, FieldOfView
    )
    
    # Convert capabilities
    capabilities = [SensorCapability(cap) for cap in sensor.capabilities]
    
    # Convert resolution if exists
    resolution = Resolution(**sensor.resolution) if sensor.resolution else None
    
    # Convert field of view if exists
    field_of_view = FieldOfView(**sensor.field_of_view) if sensor.field_of_view else None
    
    # Convert to schema
    return Sensor(
        id=sensor.id,
        name=sensor.name,
        type=SensorType(sensor.type),
        platform_id=sensor.platform_id,
        capabilities=capabilities,
        resolution=resolution,
        field_of_view=field_of_view,
        range=sensor.range,
        accuracy=sensor.accuracy,
        status=SensorStatus(sensor.status),
        metadata=sensor.metadata,
        created_at=sensor.created_at,
        updated_at=sensor.updated_at
    )
