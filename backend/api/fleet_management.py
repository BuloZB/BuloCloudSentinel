"""
API endpoints for fleet management.

This module provides FastAPI endpoints for managing fleets of drones,
including formations, behaviors, and fleet missions.
"""

from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks

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
from pydantic import BaseModel
from typing import List, Dict, Any, Optional

from backend.fleet_management.models import (
    Fleet, FleetCreate, FleetResponse,
    Formation, FormationCreate, FormationResponse,
    Behavior, BehaviorCreate, BehaviorResponse,
    FleetMission, FleetMissionCreate, FleetMissionResponse,
    FleetDrone, DroneRole
)
from backend.fleet_management.coordinator import FleetCoordinatorService
from backend.api.dependencies import get_current_user

router = APIRouter(prefix="/fleet", tags=["Fleet Management"])

# Global service instance
coordinator_service = None


# Initialize service

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

def initialize_service(service: FleetCoordinatorService):
    """Initialize the fleet coordinator service."""
    global coordinator_service
    coordinator_service = service


# API endpoints
@router.post("/create", response_model=FleetResponse)
async def create_fleet(
    request: FleetCreate,
    current_user: str = Depends(get_current_user)
):
    """Create a new fleet."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    # Create fleet
    fleet = Fleet(
        name=request.name,
        description=request.description,
        drones=request.drones
    )
    
    created_fleet = await coordinator_service.create_fleet(fleet)
    return FleetResponse(**created_fleet.dict())


@router.get("/{fleet_id}", response_model=FleetResponse)
async def get_fleet(
    fleet_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get a fleet by ID."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    fleet = await coordinator_service.get_fleet(fleet_id)
    if not fleet:
        raise HTTPException(status_code=404, detail=f"Fleet {fleet_id} not found")
    
    return FleetResponse(**fleet.dict())


@router.get("/", response_model=List[FleetResponse])
async def list_fleets(
    current_user: str = Depends(get_current_user)
):
    """List all fleets."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    fleets = await coordinator_service.list_fleets()
    return [FleetResponse(**fleet.dict()) for fleet in fleets]


@router.put("/{fleet_id}/add-drone", response_model=FleetResponse)
async def add_drone_to_fleet(
    fleet_id: str,
    drone: FleetDrone,
    current_user: str = Depends(get_current_user)
):
    """Add a drone to a fleet."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    fleet = await coordinator_service.add_drone_to_fleet(fleet_id, drone)
    if not fleet:
        raise HTTPException(status_code=404, detail=f"Fleet {fleet_id} not found")
    
    return FleetResponse(**fleet.dict())


@router.delete("/{fleet_id}/remove-drone/{drone_id}", response_model=FleetResponse)
async def remove_drone_from_fleet(
    fleet_id: str,
    drone_id: str,
    current_user: str = Depends(get_current_user)
):
    """Remove a drone from a fleet."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    fleet = await coordinator_service.remove_drone_from_fleet(fleet_id, drone_id)
    if not fleet:
        raise HTTPException(status_code=404, detail=f"Fleet {fleet_id} not found")
    
    return FleetResponse(**fleet.dict())


@router.post("/formation/create", response_model=FormationResponse)
async def create_formation(
    request: FormationCreate,
    current_user: str = Depends(get_current_user)
):
    """Create a new formation."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    # Create formation
    formation = Formation(
        name=request.name,
        type=request.type,
        parameters=request.parameters,
        drone_positions=request.drone_positions or {}
    )
    
    created_formation = await coordinator_service.create_formation(formation)
    return FormationResponse(**created_formation.dict())


@router.get("/formation/{formation_id}", response_model=FormationResponse)
async def get_formation(
    formation_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get a formation by ID."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    formation = await coordinator_service.get_formation(formation_id)
    if not formation:
        raise HTTPException(status_code=404, detail=f"Formation {formation_id} not found")
    
    return FormationResponse(**formation.dict())


@router.get("/formation", response_model=List[FormationResponse])
async def list_formations(
    current_user: str = Depends(get_current_user)
):
    """List all formations."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    formations = await coordinator_service.list_formations()
    return [FormationResponse(**formation.dict()) for formation in formations]


@router.post("/behavior/create", response_model=BehaviorResponse)
async def create_behavior(
    request: BehaviorCreate,
    current_user: str = Depends(get_current_user)
):
    """Create a new behavior."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    # Create behavior
    behavior = Behavior(
        name=request.name,
        type=request.type,
        parameters=request.parameters
    )
    
    created_behavior = await coordinator_service.create_behavior(behavior)
    return BehaviorResponse(**created_behavior.dict())


@router.get("/behavior/{behavior_id}", response_model=BehaviorResponse)
async def get_behavior(
    behavior_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get a behavior by ID."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    behavior = await coordinator_service.get_behavior(behavior_id)
    if not behavior:
        raise HTTPException(status_code=404, detail=f"Behavior {behavior_id} not found")
    
    return BehaviorResponse(**behavior.dict())


@router.get("/behavior", response_model=List[BehaviorResponse])
async def list_behaviors(
    current_user: str = Depends(get_current_user)
):
    """List all behaviors."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    behaviors = await coordinator_service.list_behaviors()
    return [BehaviorResponse(**behavior.dict()) for behavior in behaviors]


@router.post("/mission/create", response_model=FleetMissionResponse)
async def create_fleet_mission(
    request: FleetMissionCreate,
    current_user: str = Depends(get_current_user)
):
    """Create a new fleet mission."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    # Create fleet mission
    fleet_mission = FleetMission(
        name=request.name,
        description=request.description,
        fleet_id=request.fleet_id,
        formation_id=request.formation_id,
        behavior_id=request.behavior_id,
        drone_missions=request.individual_missions or {}
    )
    
    created_mission = await coordinator_service.create_fleet_mission(fleet_mission)
    return FleetMissionResponse(**created_mission.dict())


@router.get("/mission/{mission_id}", response_model=FleetMissionResponse)
async def get_fleet_mission(
    mission_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get a fleet mission by ID."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    mission = await coordinator_service.get_fleet_mission(mission_id)
    if not mission:
        raise HTTPException(status_code=404, detail=f"Fleet mission {mission_id} not found")
    
    return FleetMissionResponse(**mission.dict())


@router.get("/mission", response_model=List[FleetMissionResponse])
async def list_fleet_missions(
    fleet_id: Optional[str] = None,
    current_user: str = Depends(get_current_user)
):
    """List fleet missions."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    missions = await coordinator_service.list_fleet_missions(fleet_id)
    return [FleetMissionResponse(**mission.dict()) for mission in missions]


@router.post("/mission/{mission_id}/execute")
async def execute_fleet_mission(
    mission_id: str,
    current_user: str = Depends(get_current_user)
):
    """Execute a fleet mission."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    success = await coordinator_service.execute_fleet_mission(mission_id)
    if not success:
        raise HTTPException(status_code=400, detail=f"Failed to execute fleet mission {mission_id}")
    
    return {"message": f"Fleet mission {mission_id} execution started"}


@router.post("/mission/{mission_id}/abort")
async def abort_fleet_mission(
    mission_id: str,
    current_user: str = Depends(get_current_user)
):
    """Abort a fleet mission."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    success = await coordinator_service.abort_fleet_mission(mission_id)
    if not success:
        raise HTTPException(status_code=400, detail=f"Failed to abort fleet mission {mission_id}")
    
    return {"message": f"Fleet mission {mission_id} aborted"}


@router.get("/{fleet_id}/telemetry")
async def get_fleet_telemetry(
    fleet_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get telemetry for a fleet."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    try:
        telemetry = await coordinator_service.get_fleet_telemetry(fleet_id)
        return telemetry
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting fleet telemetry: {str(e)}")


@router.post("/start-service")
async def start_service(
    background_tasks: BackgroundTasks,
    current_user: str = Depends(get_current_user)
):
    """Start the fleet coordinator service."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    background_tasks.add_task(coordinator_service.start)
    return {"message": "Fleet coordinator service starting"}


@router.post("/stop-service")
async def stop_service(
    background_tasks: BackgroundTasks,
    current_user: str = Depends(get_current_user)
):
    """Stop the fleet coordinator service."""
    if not coordinator_service:
        raise HTTPException(status_code=500, detail="Fleet coordinator service not initialized")
    
    background_tasks.add_task(coordinator_service.stop)
    return {"message": "Fleet coordinator service stopping"}
