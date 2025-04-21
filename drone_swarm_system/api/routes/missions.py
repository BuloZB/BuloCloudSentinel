"""
Mission API routes for the Drone Swarm System.

This module provides endpoints for mission planning and execution.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, BackgroundTasks
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from datetime import datetime

# Import local modules
from services.mission_service import MissionService
from api.schemas.mission import (
    Mission,
    MissionCreate,
    MissionUpdate,
    MissionState,
    Waypoint,
    MissionType
)

router = APIRouter()

# Models
class ExecuteMissionRequest(BaseModel):
    """Request model for executing a mission."""
    drone_ids: List[str]
    start_time: Optional[datetime] = None
    parameters: Optional[Dict[str, Any]] = None

class PauseMissionRequest(BaseModel):
    """Request model for pausing a mission."""
    reason: Optional[str] = None

class ResumeMissionRequest(BaseModel):
    """Request model for resuming a mission."""
    parameters: Optional[Dict[str, Any]] = None

class AbortMissionRequest(BaseModel):
    """Request model for aborting a mission."""
    reason: str
    emergency: bool = False

# Helper functions
def get_mission_service(request: Request) -> MissionService:
    """Get the mission service from the app state."""
    return request.app.state.mission_service

# Routes
@router.get("/")
async def get_all_missions(
    status: Optional[str] = None,
    mission_type: Optional[str] = None,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Get all missions, optionally filtered by status and type."""
    try:
        missions = await mission_service.get_all_missions(status, mission_type)
        return missions
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting missions: {str(e)}")

@router.get("/{mission_id}")
async def get_mission(
    mission_id: str,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Get a specific mission by ID."""
    try:
        mission = await mission_service.get_mission(mission_id)
        if not mission:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        return mission
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting mission: {str(e)}")

@router.post("/")
async def create_mission(
    mission: MissionCreate,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Create a new mission."""
    try:
        new_mission = await mission_service.create_mission(mission)
        return new_mission
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating mission: {str(e)}")

@router.put("/{mission_id}")
async def update_mission(
    mission_id: str,
    mission: MissionUpdate,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Update a mission."""
    try:
        updated_mission = await mission_service.update_mission(mission_id, mission)
        if not updated_mission:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        return updated_mission
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating mission: {str(e)}")

@router.delete("/{mission_id}")
async def delete_mission(
    mission_id: str,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Delete a mission."""
    try:
        success = await mission_service.delete_mission(mission_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting mission: {str(e)}")

@router.get("/{mission_id}/state")
async def get_mission_state(
    mission_id: str,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Get the current state of a mission."""
    try:
        state = await mission_service.get_mission_state(mission_id)
        if not state:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        return state
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting mission state: {str(e)}")

@router.post("/{mission_id}/execute")
async def execute_mission(
    mission_id: str,
    request: ExecuteMissionRequest,
    background_tasks: BackgroundTasks,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Execute a mission."""
    try:
        # Check if mission exists
        mission = await mission_service.get_mission(mission_id)
        if not mission:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        
        # Execute mission in background
        background_tasks.add_task(
            mission_service.execute_mission,
            mission_id,
            request.drone_ids,
            request.start_time,
            request.parameters
        )
        
        return {"message": f"Mission {mission_id} execution started"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error executing mission: {str(e)}")

@router.post("/{mission_id}/pause")
async def pause_mission(
    mission_id: str,
    request: PauseMissionRequest,
    background_tasks: BackgroundTasks,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Pause a mission."""
    try:
        # Check if mission exists
        mission = await mission_service.get_mission(mission_id)
        if not mission:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        
        # Pause mission in background
        background_tasks.add_task(
            mission_service.pause_mission,
            mission_id,
            request.reason
        )
        
        return {"message": f"Mission {mission_id} pause requested"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error pausing mission: {str(e)}")

@router.post("/{mission_id}/resume")
async def resume_mission(
    mission_id: str,
    request: ResumeMissionRequest,
    background_tasks: BackgroundTasks,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Resume a paused mission."""
    try:
        # Check if mission exists
        mission = await mission_service.get_mission(mission_id)
        if not mission:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        
        # Resume mission in background
        background_tasks.add_task(
            mission_service.resume_mission,
            mission_id,
            request.parameters
        )
        
        return {"message": f"Mission {mission_id} resume requested"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error resuming mission: {str(e)}")

@router.post("/{mission_id}/abort")
async def abort_mission(
    mission_id: str,
    request: AbortMissionRequest,
    background_tasks: BackgroundTasks,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Abort a mission."""
    try:
        # Check if mission exists
        mission = await mission_service.get_mission(mission_id)
        if not mission:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        
        # Abort mission in background
        background_tasks.add_task(
            mission_service.abort_mission,
            mission_id,
            request.reason,
            request.emergency
        )
        
        return {"message": f"Mission {mission_id} abort requested"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error aborting mission: {str(e)}")

@router.get("/{mission_id}/logs")
async def get_mission_logs(
    mission_id: str,
    limit: int = 100,
    level: Optional[str] = None,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Get logs for a mission."""
    try:
        logs = await mission_service.get_mission_logs(mission_id, limit, level)
        return logs
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting mission logs: {str(e)}")

@router.get("/types")
async def get_mission_types(
    mission_service: MissionService = Depends(get_mission_service)
):
    """Get all available mission types."""
    try:
        types = await mission_service.get_mission_types()
        return types
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting mission types: {str(e)}")

@router.post("/{mission_id}/simulate")
async def simulate_mission(
    mission_id: str,
    background_tasks: BackgroundTasks,
    mission_service: MissionService = Depends(get_mission_service)
):
    """Simulate a mission without actually executing it."""
    try:
        # Check if mission exists
        mission = await mission_service.get_mission(mission_id)
        if not mission:
            raise HTTPException(status_code=404, detail=f"Mission {mission_id} not found")
        
        # Simulate mission in background
        background_tasks.add_task(mission_service.simulate_mission, mission_id)
        
        return {"message": f"Mission {mission_id} simulation started"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error simulating mission: {str(e)}")
