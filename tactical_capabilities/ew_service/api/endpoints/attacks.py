"""
API endpoints for electronic attack management.
"""

from typing import List, Optional, Dict, Any

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
from sqlalchemy import update, delete
from datetime import datetime

from api.schemas import (
    ElectronicAttack, ElectronicAttackCreate, ElectronicAttackUpdate,
    AttackType
)
from core.security import get_current_user, has_permission, check_clearance, log_security_event
from db.models import ElectronicAttack as ElectronicAttackModel
from db.session import get_db_session
from services.electronic_attack import ElectronicAttack as ElectronicAttackService

router = APIRouter()

@router.get("/", response_model=List[ElectronicAttack])
async def get_attacks(
    skip: int = 0,
    limit: int = 100,
    platform_id: Optional[UUID] = None,
    attack_type: Optional[str] = None,
    status: Optional[str] = None,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get all electronic attacks with optional filtering.
    
    Args:
        skip: Number of attacks to skip
        limit: Maximum number of attacks to return
        platform_id: Filter by platform ID
        attack_type: Filter by attack type
        status: Filter by attack status
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        List of electronic attacks
    """
    query = select(ElectronicAttackModel)
    
    # Apply filters
    if platform_id:
        query = query.filter(ElectronicAttackModel.platform_id == platform_id)
    if attack_type:
        query = query.filter(ElectronicAttackModel.attack_type == attack_type)
    if status:
        query = query.filter(ElectronicAttackModel.status == status)
    
    # Apply pagination
    query = query.offset(skip).limit(limit)
    
    # Execute query
    result = await db.execute(query)
    attacks = result.scalars().all()
    
    return attacks

@router.post("/", response_model=ElectronicAttack, status_code=status.HTTP_201_CREATED)
async def create_attack(
    attack: ElectronicAttackCreate,
    request: Request,
    current_user = Depends(has_permission("ew:create_attack")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Create a new electronic attack.
    
    Args:
        attack: Electronic attack data
        request: FastAPI request
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Created electronic attack
    """
    # Check if user has required clearance
    if attack.attack_type in ["jamming", "spoofing"] and not check_clearance(current_user, "secret"):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Insufficient clearance level for this operation"
        )
    
    # Get platform
    platform_id = attack.platform_id
    
    # Create attack model
    db_attack = ElectronicAttackModel(
        platform_id=attack.platform_id,
        target_id=attack.target_id,
        attack_type=attack.attack_type,
        frequency=attack.frequency,
        bandwidth=attack.bandwidth,
        power=attack.power,
        waveform_id=attack.waveform_id,
        start_time=attack.start_time,
        end_time=attack.end_time,
        status="scheduled",
        location=attack.location.dict() if attack.location else None,
        direction=attack.direction.dict() if attack.direction else None,
        metadata=attack.metadata
    )
    
    # Add to database
    db.add(db_attack)
    await db.commit()
    await db.refresh(db_attack)
    
    # Log the operation
    await log_security_event(
        event_type="electronic_attack_created",
        user_id=current_user.id,
        resource_id=str(db_attack.id),
        resource_type="electronic_attack",
        details={
            "attack_type": attack.attack_type,
            "platform_id": str(attack.platform_id),
            "frequency": attack.frequency,
            "power": attack.power
        }
    )
    
    # Schedule attack execution
    await request.app.state.attack_manager.create_attack(
        platform_id=str(attack.platform_id),
        attack_type=attack.attack_type,
        frequency=attack.frequency,
        bandwidth=attack.bandwidth,
        power=attack.power,
        duration=int((attack.end_time - attack.start_time).total_seconds()),
        waveform_id=str(attack.waveform_id) if attack.waveform_id else None,
        target_id=str(attack.target_id) if attack.target_id else None,
        direction=attack.direction.dict() if attack.direction else None,
        metadata=attack.metadata
    )
    
    return db_attack

@router.post("/jamming", status_code=status.HTTP_201_CREATED)
async def execute_jamming_attack(
    request: Request,
    platform_id: UUID,
    frequency: float,
    bandwidth: float,
    power: float,
    duration: int,
    jamming_type: str = "noise",
    jamming_subtype: str = "white",
    waveform_params: Optional[Dict[str, Any]] = Body(None),
    direction: Optional[Dict[str, float]] = Body(None),
    target_id: Optional[UUID] = None,
    metadata: Optional[Dict[str, Any]] = Body(None),
    current_user = Depends(has_permission("ew:execute_attack"))
):
    """
    Execute a jamming attack.
    
    Args:
        request: FastAPI request
        platform_id: EW platform ID
        frequency: Center frequency in Hz
        bandwidth: Bandwidth in Hz
        power: Power in Watts
        duration: Duration in seconds
        jamming_type: Type of jamming (noise, tone, sweep, pulse, chirp)
        jamming_subtype: Subtype of jamming (e.g., white, pink for noise)
        waveform_params: Additional waveform parameters
        direction: Optional direction (azimuth, elevation)
        target_id: Optional target ID
        metadata: Optional metadata
        current_user: Current authenticated user with required permission
        
    Returns:
        Attack information
    """
    # Check if user has required clearance
    if not check_clearance(current_user, "secret"):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Insufficient clearance level for this operation"
        )
    
    # Get electronic attack service
    electronic_attack_service = ElectronicAttackService()
    
    try:
        # Execute jamming attack
        attack_info = await electronic_attack_service.execute_jamming_attack(
            platform_id=str(platform_id),
            frequency=frequency,
            bandwidth=bandwidth,
            power=power,
            duration=duration,
            jamming_type=jamming_type,
            jamming_subtype=jamming_subtype,
            waveform_params=waveform_params,
            direction=direction,
            target_id=str(target_id) if target_id else None,
            metadata=metadata
        )
        
        # Log the operation
        await log_security_event(
            event_type="jamming_attack_executed",
            user_id=current_user.id,
            resource_id=attack_info["id"],
            resource_type="electronic_attack",
            details={
                "jamming_type": jamming_type,
                "jamming_subtype": jamming_subtype,
                "platform_id": str(platform_id),
                "frequency": frequency,
                "power": power,
                "duration": duration
            }
        )
        
        return attack_info
    
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to execute jamming attack: {str(e)}"
        )

@router.post("/spoofing", status_code=status.HTTP_201_CREATED)
async def execute_spoofing_attack(
    request: Request,
    platform_id: UUID,
    frequency: float,
    bandwidth: float,
    power: float,
    duration: int,
    spoofing_type: str,
    spoofing_params: Dict[str, Any],
    direction: Optional[Dict[str, float]] = Body(None),
    target_id: Optional[UUID] = None,
    metadata: Optional[Dict[str, Any]] = Body(None),
    current_user = Depends(has_permission("ew:execute_attack"))
):
    """
    Execute a spoofing attack.
    
    Args:
        request: FastAPI request
        platform_id: EW platform ID
        frequency: Center frequency in Hz
        bandwidth: Bandwidth in Hz
        power: Power in Watts
        duration: Duration in seconds
        spoofing_type: Type of spoofing (gps, radar, communications)
        spoofing_params: Spoofing parameters
        direction: Optional direction (azimuth, elevation)
        target_id: Optional target ID
        metadata: Optional metadata
        current_user: Current authenticated user with required permission
        
    Returns:
        Attack information
    """
    # Check if user has required clearance
    if not check_clearance(current_user, "top_secret"):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Insufficient clearance level for this operation"
        )
    
    # Get electronic attack service
    electronic_attack_service = ElectronicAttackService()
    
    try:
        # Execute spoofing attack
        attack_info = await electronic_attack_service.execute_spoofing_attack(
            platform_id=str(platform_id),
            frequency=frequency,
            bandwidth=bandwidth,
            power=power,
            duration=duration,
            spoofing_type=spoofing_type,
            spoofing_params=spoofing_params,
            direction=direction,
            target_id=str(target_id) if target_id else None,
            metadata=metadata
        )
        
        # Log the operation
        await log_security_event(
            event_type="spoofing_attack_executed",
            user_id=current_user.id,
            resource_id=attack_info["id"],
            resource_type="electronic_attack",
            details={
                "spoofing_type": spoofing_type,
                "platform_id": str(platform_id),
                "frequency": frequency,
                "power": power,
                "duration": duration
            }
        )
        
        return attack_info
    
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to execute spoofing attack: {str(e)}"
        )

@router.post("/deception", status_code=status.HTTP_201_CREATED)
async def execute_deception_attack(
    request: Request,
    platform_id: UUID,
    frequency: float,
    bandwidth: float,
    power: float,
    duration: int,
    deception_type: str,
    deception_params: Dict[str, Any],
    direction: Optional[Dict[str, float]] = Body(None),
    target_id: Optional[UUID] = None,
    metadata: Optional[Dict[str, Any]] = Body(None),
    current_user = Depends(has_permission("ew:execute_attack"))
):
    """
    Execute a deception attack.
    
    Args:
        request: FastAPI request
        platform_id: EW platform ID
        frequency: Center frequency in Hz
        bandwidth: Bandwidth in Hz
        power: Power in Watts
        duration: Duration in seconds
        deception_type: Type of deception
        deception_params: Deception parameters
        direction: Optional direction (azimuth, elevation)
        target_id: Optional target ID
        metadata: Optional metadata
        current_user: Current authenticated user with required permission
        
    Returns:
        Attack information
    """
    # Check if user has required clearance
    if not check_clearance(current_user, "top_secret"):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Insufficient clearance level for this operation"
        )
    
    # Get electronic attack service
    electronic_attack_service = ElectronicAttackService()
    
    try:
        # Execute deception attack
        attack_info = await electronic_attack_service.execute_deception_attack(
            platform_id=str(platform_id),
            frequency=frequency,
            bandwidth=bandwidth,
            power=power,
            duration=duration,
            deception_type=deception_type,
            deception_params=deception_params,
            direction=direction,
            target_id=str(target_id) if target_id else None,
            metadata=metadata
        )
        
        # Log the operation
        await log_security_event(
            event_type="deception_attack_executed",
            user_id=current_user.id,
            resource_id=attack_info["id"],
            resource_type="electronic_attack",
            details={
                "deception_type": deception_type,
                "platform_id": str(platform_id),
                "frequency": frequency,
                "power": power,
                "duration": duration
            }
        )
        
        return attack_info
    
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to execute deception attack: {str(e)}"
        )

@router.post("/meaconing", status_code=status.HTTP_201_CREATED)
async def execute_meaconing_attack(
    request: Request,
    platform_id: UUID,
    frequency: float,
    bandwidth: float,
    power: float,
    duration: int,
    delay: float,
    amplification: float = 1.0,
    direction: Optional[Dict[str, float]] = Body(None),
    target_id: Optional[UUID] = None,
    metadata: Optional[Dict[str, Any]] = Body(None),
    current_user = Depends(has_permission("ew:execute_attack"))
):
    """
    Execute a meaconing attack (intercept and rebroadcast).
    
    Args:
        request: FastAPI request
        platform_id: EW platform ID
        frequency: Center frequency in Hz
        bandwidth: Bandwidth in Hz
        power: Power in Watts
        duration: Duration in seconds
        delay: Delay in seconds
        amplification: Signal amplification factor
        direction: Optional direction (azimuth, elevation)
        target_id: Optional target ID
        metadata: Optional metadata
        current_user: Current authenticated user with required permission
        
    Returns:
        Attack information
    """
    # Check if user has required clearance
    if not check_clearance(current_user, "top_secret"):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Insufficient clearance level for this operation"
        )
    
    # Get electronic attack service
    electronic_attack_service = ElectronicAttackService()
    
    try:
        # Execute meaconing attack
        attack_info = await electronic_attack_service.execute_meaconing_attack(
            platform_id=str(platform_id),
            frequency=frequency,
            bandwidth=bandwidth,
            power=power,
            duration=duration,
            delay=delay,
            amplification=amplification,
            direction=direction,
            target_id=str(target_id) if target_id else None,
            metadata=metadata
        )
        
        # Log the operation
        await log_security_event(
            event_type="meaconing_attack_executed",
            user_id=current_user.id,
            resource_id=attack_info["id"],
            resource_type="electronic_attack",
            details={
                "platform_id": str(platform_id),
                "frequency": frequency,
                "power": power,
                "duration": duration,
                "delay": delay,
                "amplification": amplification
            }
        )
        
        return attack_info
    
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to execute meaconing attack: {str(e)}"
        )

@router.get("/{attack_id}", response_model=ElectronicAttack)
async def get_attack(
    attack_id: UUID,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get a specific electronic attack by ID.
    
    Args:
        attack_id: Electronic attack ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Electronic attack
        
    Raises:
        HTTPException: If attack not found
    """
    # Get attack
    result = await db.execute(select(ElectronicAttackModel).filter(ElectronicAttackModel.id == attack_id))
    attack = result.scalars().first()
    
    # Check if attack exists
    if not attack:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Electronic attack with ID {attack_id} not found"
        )
    
    return attack

@router.put("/{attack_id}", response_model=ElectronicAttack)
async def update_attack(
    attack_id: UUID,
    attack_update: ElectronicAttackUpdate,
    request: Request,
    current_user = Depends(has_permission("ew:update_attack")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Update an electronic attack.
    
    Args:
        attack_id: Electronic attack ID
        attack_update: Electronic attack update data
        request: FastAPI request
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Updated electronic attack
        
    Raises:
        HTTPException: If attack not found
    """
    # Get attack
    result = await db.execute(select(ElectronicAttackModel).filter(ElectronicAttackModel.id == attack_id))
    db_attack = result.scalars().first()
    
    # Check if attack exists
    if not db_attack:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Electronic attack with ID {attack_id} not found"
        )
    
    # Check if attack can be updated
    if db_attack.status not in ["scheduled", "active"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Cannot update attack with status {db_attack.status}"
        )
    
    # Update attack fields
    update_data = attack_update.dict(exclude_unset=True)
    
    # Handle nested objects
    if "location" in update_data and update_data["location"]:
        update_data["location"] = update_data["location"].dict()
    if "direction" in update_data and update_data["direction"]:
        update_data["direction"] = update_data["direction"].dict()
    
    # Update attack
    for key, value in update_data.items():
        setattr(db_attack, key, value)
    
    # Commit changes
    await db.commit()
    await db.refresh(db_attack)
    
    # Update attack in attack manager if needed
    if "end_time" in update_data or "status" in update_data:
        if update_data.get("status") == "cancelled":
            await request.app.state.attack_manager.cancel_attack(str(attack_id))
    
    # Log the operation
    await log_security_event(
        event_type="electronic_attack_updated",
        user_id=current_user.id,
        resource_id=str(db_attack.id),
        resource_type="electronic_attack",
        details={
            "updated_fields": list(update_data.keys())
        }
    )
    
    return db_attack

@router.delete("/{attack_id}", status_code=status.HTTP_204_NO_CONTENT)
async def cancel_attack(
    attack_id: UUID,
    request: Request,
    current_user = Depends(has_permission("ew:cancel_attack")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Cancel an electronic attack.
    
    Args:
        attack_id: Electronic attack ID
        request: FastAPI request
        current_user: Current authenticated user with required permission
        db: Database session
        
    Raises:
        HTTPException: If attack not found or cannot be cancelled
    """
    # Get attack
    result = await db.execute(select(ElectronicAttackModel).filter(ElectronicAttackModel.id == attack_id))
    db_attack = result.scalars().first()
    
    # Check if attack exists
    if not db_attack:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Electronic attack with ID {attack_id} not found"
        )
    
    # Check if attack can be cancelled
    if db_attack.status not in ["scheduled", "active"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Cannot cancel attack with status {db_attack.status}"
        )
    
    # Update attack status
    db_attack.status = "cancelled"
    db_attack.end_time = datetime.utcnow()
    
    # Commit changes
    await db.commit()
    
    # Cancel attack in attack manager
    await request.app.state.attack_manager.cancel_attack(str(attack_id))
    
    # Log the operation
    await log_security_event(
        event_type="electronic_attack_cancelled",
        user_id=current_user.id,
        resource_id=str(attack_id),
        resource_type="electronic_attack",
        details={
            "platform_id": str(db_attack.platform_id),
            "attack_type": db_attack.attack_type
        }
    )
    
    return None
