"""
API endpoints for coordination planning.
"""

from typing import List, Optional, Dict, Any
from uuid import UUID
from fastapi import APIRouter, Depends, HTTPException, Query, status, Request, Body
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from datetime import datetime

from api.schemas import (
    CoordinationPlan, CoordinationPlanCreate, CoordinationPlanUpdate,
    PlanStatus, Waypoint
)
from core.security import get_current_user, has_permission, log_security_event
from db.session import get_db_session
from db.models import CoordinationPlan as CoordinationPlanModel, Target as TargetModel

router = APIRouter()

@router.post("/plan", response_model=CoordinationPlan, status_code=status.HTTP_201_CREATED)
async def create_coordination_plan(
    plan: CoordinationPlanCreate,
    current_user = Depends(has_permission("tacs:create_plan")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Create a new coordination plan.
    
    Args:
        plan: Coordination plan data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Created coordination plan
    """
    # Create coordination plan
    db_plan = CoordinationPlanModel(
        name=plan.name,
        start_time=plan.start_time,
        end_time=plan.end_time,
        waypoints=plan.waypoints,
        sensor_configurations=plan.sensor_configurations,
        priority=plan.priority,
        status=plan.status.value,
        metadata=plan.metadata
    )
    
    # Add to database
    db.add(db_plan)
    await db.commit()
    await db.refresh(db_plan)
    
    # Associate targets
    for target_id in plan.target_ids:
        # Get target
        result = await db.execute(select(TargetModel).filter(TargetModel.id == target_id))
        target = result.scalars().first()
        
        if target:
            # Add association
            db_plan.targets.append(target)
    
    # Commit changes
    await db.commit()
    await db.refresh(db_plan)
    
    # Log the operation
    await log_security_event(
        event_type="coordination_plan_created",
        user_id=current_user.id,
        resource_id=str(db_plan.id),
        resource_type="coordination_plan",
        details={
            "name": plan.name,
            "target_ids": [str(id) for id in plan.target_ids],
            "platform_ids": [str(id) for id in plan.platform_ids],
            "status": plan.status.value
        }
    )
    
    # Convert to schema
    return _convert_to_plan_schema(db_plan)

@router.get("/plan/{plan_id}", response_model=CoordinationPlan)
async def get_coordination_plan(
    plan_id: UUID,
    current_user = Depends(get_current_user),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get a specific coordination plan by ID.
    
    Args:
        plan_id: Coordination plan ID
        current_user: Current authenticated user
        db: Database session
        
    Returns:
        Coordination plan
        
    Raises:
        HTTPException: If plan not found
    """
    # Get plan
    result = await db.execute(select(CoordinationPlanModel).filter(CoordinationPlanModel.id == plan_id))
    plan = result.scalars().first()
    
    # Check if plan exists
    if not plan:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Coordination plan with ID {plan_id} not found"
        )
    
    # Convert to schema
    return _convert_to_plan_schema(plan)

@router.put("/plan/{plan_id}", response_model=CoordinationPlan)
async def update_coordination_plan(
    plan_id: UUID,
    plan_update: CoordinationPlanUpdate,
    current_user = Depends(has_permission("tacs:update_plan")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Update a coordination plan.
    
    Args:
        plan_id: Coordination plan ID
        plan_update: Coordination plan update data
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Updated coordination plan
        
    Raises:
        HTTPException: If plan not found
    """
    # Get plan
    result = await db.execute(select(CoordinationPlanModel).filter(CoordinationPlanModel.id == plan_id))
    plan = result.scalars().first()
    
    # Check if plan exists
    if not plan:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Coordination plan with ID {plan_id} not found"
        )
    
    # Update fields
    update_data = plan_update.dict(exclude_unset=True)
    
    # Handle enums
    if "status" in update_data and update_data["status"]:
        update_data["status"] = update_data["status"].value
    
    # Handle target_ids separately
    target_ids = update_data.pop("target_ids", None)
    
    # Handle platform_ids separately
    platform_ids = update_data.pop("platform_ids", None)
    
    # Update plan
    for key, value in update_data.items():
        setattr(plan, key, value)
    
    # Update targets if provided
    if target_ids is not None:
        # Clear existing associations
        plan.targets = []
        
        # Add new associations
        for target_id in target_ids:
            # Get target
            result = await db.execute(select(TargetModel).filter(TargetModel.id == target_id))
            target = result.scalars().first()
            
            if target:
                # Add association
                plan.targets.append(target)
    
    # Update platform_ids if provided
    if platform_ids is not None:
        # Update platform associations in the association table
        # This is a simplified implementation
        # In a real system, this would update the association table
        pass
    
    # Commit changes
    await db.commit()
    await db.refresh(plan)
    
    # Log the operation
    await log_security_event(
        event_type="coordination_plan_updated",
        user_id=current_user.id,
        resource_id=str(plan_id),
        resource_type="coordination_plan",
        details={
            "updated_fields": [k for k, v in update_data.items() if v is not None]
        }
    )
    
    # Convert to schema
    return _convert_to_plan_schema(plan)

@router.delete("/plan/{plan_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_coordination_plan(
    plan_id: UUID,
    current_user = Depends(has_permission("tacs:delete_plan")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Delete a coordination plan.
    
    Args:
        plan_id: Coordination plan ID
        current_user: Current authenticated user with required permission
        db: Database session
        
    Raises:
        HTTPException: If plan not found
    """
    # Get plan
    result = await db.execute(select(CoordinationPlanModel).filter(CoordinationPlanModel.id == plan_id))
    plan = result.scalars().first()
    
    # Check if plan exists
    if not plan:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Coordination plan with ID {plan_id} not found"
        )
    
    # Delete plan
    await db.delete(plan)
    await db.commit()
    
    # Log the operation
    await log_security_event(
        event_type="coordination_plan_deleted",
        user_id=current_user.id,
        resource_id=str(plan_id),
        resource_type="coordination_plan",
        details={}
    )
    
    return None

@router.post("/execute/{plan_id}")
async def execute_coordination_plan(
    plan_id: UUID,
    current_user = Depends(has_permission("tacs:execute_plan")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Execute a coordination plan.
    
    Args:
        plan_id: Coordination plan ID
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Execution result
        
    Raises:
        HTTPException: If plan not found or cannot be executed
    """
    # Get plan
    result = await db.execute(select(CoordinationPlanModel).filter(CoordinationPlanModel.id == plan_id))
    plan = result.scalars().first()
    
    # Check if plan exists
    if not plan:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Coordination plan with ID {plan_id} not found"
        )
    
    # Check if plan can be executed
    if plan.status not in ["draft", "approved"]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Cannot execute plan with status {plan.status}"
        )
    
    # Update plan status
    plan.status = "executing"
    await db.commit()
    
    # In a real implementation, this would send commands to platforms
    # For now, we'll just simulate it
    
    # Log the operation
    await log_security_event(
        event_type="coordination_plan_executed",
        user_id=current_user.id,
        resource_id=str(plan_id),
        resource_type="coordination_plan",
        details={}
    )
    
    # Return result
    return {
        "message": f"Executing coordination plan {plan_id}",
        "plan_id": str(plan_id),
        "status": plan.status
    }

@router.post("/generate")
async def generate_coordination_plan(
    target_ids: List[UUID],
    platform_ids: List[UUID],
    start_time: datetime,
    end_time: Optional[datetime] = None,
    name: Optional[str] = None,
    priority: int = Body(1, ge=1, le=10),
    parameters: Optional[Dict[str, Any]] = Body(None),
    current_user = Depends(has_permission("tacs:create_plan")),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Generate a coordination plan for targets and platforms.
    
    Args:
        target_ids: List of target IDs
        platform_ids: List of platform IDs
        start_time: Start time
        end_time: Optional end time
        name: Optional plan name
        priority: Plan priority
        parameters: Optional generation parameters
        current_user: Current authenticated user with required permission
        db: Database session
        
    Returns:
        Generated coordination plan
        
    Raises:
        HTTPException: If targets or platforms not found
    """
    # Get targets
    targets = []
    for target_id in target_ids:
        result = await db.execute(select(TargetModel).filter(TargetModel.id == target_id))
        target = result.scalars().first()
        
        if not target:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Target with ID {target_id} not found"
            )
        
        targets.append(target)
    
    # In a real implementation, this would get platforms from a platform service
    # For now, we'll just assume they exist
    
    # Generate plan name if not provided
    if not name:
        name = f"Plan-{uuid.uuid4().hex[:8]}"
    
    # In a real implementation, this would use a planning algorithm
    # For now, we'll just create a simple plan
    
    # Create waypoints for each platform
    waypoints = {}
    for platform_id in platform_ids:
        platform_waypoints = []
        
        # Add waypoints for each target
        for target in targets:
            # Get target location
            location = target.location
            
            # Create waypoint
            waypoint = Waypoint(
                location=GeoLocation(**location),
                altitude=100.0,
                speed=10.0,
                loiter_time=60.0,
                action="observe"
            )
            
            platform_waypoints.append(waypoint)
        
        waypoints[str(platform_id)] = platform_waypoints
    
    # Create sensor configurations
    sensor_configurations = {}
    
    # Create plan
    plan = CoordinationPlanCreate(
        name=name,
        target_ids=target_ids,
        platform_ids=platform_ids,
        start_time=start_time,
        end_time=end_time,
        waypoints={str(pid): [w.dict() for w in wp] for pid, wp in waypoints.items()},
        sensor_configurations=sensor_configurations,
        priority=priority,
        status=PlanStatus.DRAFT,
        metadata={"generated": True, "parameters": parameters or {}}
    )
    
    # Create plan in database
    db_plan = CoordinationPlanModel(
        name=plan.name,
        start_time=plan.start_time,
        end_time=plan.end_time,
        waypoints=plan.waypoints,
        sensor_configurations=plan.sensor_configurations,
        priority=plan.priority,
        status=plan.status.value,
        metadata=plan.metadata
    )
    
    # Add to database
    db.add(db_plan)
    await db.commit()
    await db.refresh(db_plan)
    
    # Associate targets
    for target in targets:
        db_plan.targets.append(target)
    
    # Commit changes
    await db.commit()
    await db.refresh(db_plan)
    
    # Log the operation
    await log_security_event(
        event_type="coordination_plan_generated",
        user_id=current_user.id,
        resource_id=str(db_plan.id),
        resource_type="coordination_plan",
        details={
            "name": plan.name,
            "target_ids": [str(id) for id in plan.target_ids],
            "platform_ids": [str(id) for id in plan.platform_ids]
        }
    )
    
    # Convert to schema
    return _convert_to_plan_schema(db_plan)

def _convert_to_plan_schema(plan: CoordinationPlanModel) -> CoordinationPlan:
    """
    Convert a coordination plan database model to a schema.
    
    Args:
        plan: Coordination plan database model
        
    Returns:
        Coordination plan schema
    """
    from api.schemas import PlanStatus, Waypoint, SensorConfiguration, GeoLocation
    
    # Convert waypoints
    waypoints = {}
    for platform_id, platform_waypoints in plan.waypoints.items():
        waypoints[platform_id] = []
        for wp_data in platform_waypoints:
            # Convert location
            location = GeoLocation(**wp_data["location"])
            
            # Create waypoint
            waypoint = Waypoint(
                location=location,
                altitude=wp_data["altitude"],
                speed=wp_data.get("speed"),
                heading=wp_data.get("heading"),
                loiter_time=wp_data.get("loiter_time"),
                arrival_time=wp_data.get("arrival_time"),
                action=wp_data.get("action"),
                sensor_actions=wp_data.get("sensor_actions")
            )
            
            waypoints[platform_id].append(waypoint)
    
    # Convert sensor configurations
    sensor_configurations = {}
    for sensor_id, config_data in plan.sensor_configurations.items():
        sensor_configurations[sensor_id] = SensorConfiguration(
            mode=config_data["mode"],
            parameters=config_data["parameters"]
        )
    
    # Convert to schema
    return CoordinationPlan(
        id=plan.id,
        name=plan.name,
        target_ids=[target.id for target in plan.targets],
        platform_ids=list(plan.waypoints.keys()),
        start_time=plan.start_time,
        end_time=plan.end_time,
        waypoints=waypoints,
        sensor_configurations=sensor_configurations,
        priority=plan.priority,
        status=PlanStatus(plan.status),
        metadata=plan.metadata,
        created_at=plan.created_at,
        updated_at=plan.updated_at
    )
