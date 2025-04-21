"""
Swarm API routes for the Drone Swarm System.

This module provides endpoints for swarm management and coordination.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, BackgroundTasks
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from datetime import datetime

# Import local modules
from coordinator.swarm_manager import SwarmManager
from api.schemas.swarm import (
    Swarm,
    SwarmCreate,
    SwarmUpdate,
    SwarmState,
    SwarmFormation,
    SwarmBehavior
)

router = APIRouter()

# Models
class FormationRequest(BaseModel):
    """Request model for setting a swarm formation."""
    formation_type: str
    parameters: Optional[Dict[str, Any]] = None

class BehaviorRequest(BaseModel):
    """Request model for setting a swarm behavior."""
    behavior_type: str
    parameters: Optional[Dict[str, Any]] = None

class TaskAllocationRequest(BaseModel):
    """Request model for allocating tasks to drones in a swarm."""
    task_type: str
    area: Optional[Dict[str, Any]] = None
    parameters: Optional[Dict[str, Any]] = None

# Helper functions
def get_swarm_manager(request: Request) -> SwarmManager:
    """Get the swarm manager from the app state."""
    return request.app.state.swarm_manager

# Routes
@router.get("/")
async def get_all_swarms(
    status: Optional[str] = None,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Get all swarms, optionally filtered by status."""
    try:
        swarms = await swarm_manager.get_all_swarms(status)
        return swarms
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting swarms: {str(e)}")

@router.get("/{swarm_id}")
async def get_swarm(
    swarm_id: str,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Get a specific swarm by ID."""
    try:
        swarm = await swarm_manager.get_swarm(swarm_id)
        if not swarm:
            raise HTTPException(status_code=404, detail=f"Swarm {swarm_id} not found")
        return swarm
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting swarm: {str(e)}")

@router.post("/")
async def create_swarm(
    swarm: SwarmCreate,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Create a new swarm."""
    try:
        new_swarm = await swarm_manager.create_swarm(swarm)
        return new_swarm
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating swarm: {str(e)}")

@router.put("/{swarm_id}")
async def update_swarm(
    swarm_id: str,
    swarm: SwarmUpdate,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Update a swarm."""
    try:
        updated_swarm = await swarm_manager.update_swarm(swarm_id, swarm)
        if not updated_swarm:
            raise HTTPException(status_code=404, detail=f"Swarm {swarm_id} not found")
        return updated_swarm
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating swarm: {str(e)}")

@router.delete("/{swarm_id}")
async def delete_swarm(
    swarm_id: str,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Delete a swarm."""
    try:
        success = await swarm_manager.delete_swarm(swarm_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Swarm {swarm_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting swarm: {str(e)}")

@router.get("/{swarm_id}/state")
async def get_swarm_state(
    swarm_id: str,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Get the current state of a swarm."""
    try:
        state = await swarm_manager.get_swarm_state(swarm_id)
        if not state:
            raise HTTPException(status_code=404, detail=f"Swarm {swarm_id} not found")
        return state
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting swarm state: {str(e)}")

@router.post("/{swarm_id}/drones/{drone_id}")
async def add_drone_to_swarm(
    swarm_id: str,
    drone_id: str,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Add a drone to a swarm."""
    try:
        success = await swarm_manager.add_drone_to_swarm(swarm_id, drone_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Swarm {swarm_id} or drone {drone_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error adding drone to swarm: {str(e)}")

@router.delete("/{swarm_id}/drones/{drone_id}")
async def remove_drone_from_swarm(
    swarm_id: str,
    drone_id: str,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Remove a drone from a swarm."""
    try:
        success = await swarm_manager.remove_drone_from_swarm(swarm_id, drone_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Swarm {swarm_id} or drone {drone_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error removing drone from swarm: {str(e)}")

@router.post("/{swarm_id}/formation")
async def set_swarm_formation(
    swarm_id: str,
    request: FormationRequest,
    background_tasks: BackgroundTasks,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Set the formation for a swarm."""
    try:
        # Check if swarm exists
        swarm = await swarm_manager.get_swarm(swarm_id)
        if not swarm:
            raise HTTPException(status_code=404, detail=f"Swarm {swarm_id} not found")
        
        # Set formation in background
        background_tasks.add_task(
            swarm_manager.set_formation,
            swarm_id,
            request.formation_type,
            request.parameters
        )
        
        return {"message": f"Formation {request.formation_type} set for swarm {swarm_id}"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error setting swarm formation: {str(e)}")

@router.post("/{swarm_id}/behavior")
async def set_swarm_behavior(
    swarm_id: str,
    request: BehaviorRequest,
    background_tasks: BackgroundTasks,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Set the behavior for a swarm."""
    try:
        # Check if swarm exists
        swarm = await swarm_manager.get_swarm(swarm_id)
        if not swarm:
            raise HTTPException(status_code=404, detail=f"Swarm {swarm_id} not found")
        
        # Set behavior in background
        background_tasks.add_task(
            swarm_manager.set_behavior,
            swarm_id,
            request.behavior_type,
            request.parameters
        )
        
        return {"message": f"Behavior {request.behavior_type} set for swarm {swarm_id}"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error setting swarm behavior: {str(e)}")

@router.post("/{swarm_id}/allocate-tasks")
async def allocate_tasks(
    swarm_id: str,
    request: TaskAllocationRequest,
    background_tasks: BackgroundTasks,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Allocate tasks to drones in a swarm."""
    try:
        # Check if swarm exists
        swarm = await swarm_manager.get_swarm(swarm_id)
        if not swarm:
            raise HTTPException(status_code=404, detail=f"Swarm {swarm_id} not found")
        
        # Allocate tasks in background
        background_tasks.add_task(
            swarm_manager.allocate_tasks,
            swarm_id,
            request.task_type,
            request.area,
            request.parameters
        )
        
        return {"message": f"Task allocation started for swarm {swarm_id}"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error allocating tasks: {str(e)}")

@router.get("/formations")
async def get_available_formations(
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Get all available swarm formations."""
    try:
        formations = await swarm_manager.get_available_formations()
        return formations
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting formations: {str(e)}")

@router.get("/behaviors")
async def get_available_behaviors(
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Get all available swarm behaviors."""
    try:
        behaviors = await swarm_manager.get_available_behaviors()
        return behaviors
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting behaviors: {str(e)}")

@router.get("/{swarm_id}/mesh-network")
async def get_mesh_network_status(
    swarm_id: str,
    swarm_manager: SwarmManager = Depends(get_swarm_manager)
):
    """Get the status of the mesh network for a swarm."""
    try:
        status = await swarm_manager.get_mesh_network_status(swarm_id)
        if not status:
            raise HTTPException(status_code=404, detail=f"Swarm {swarm_id} not found")
        return status
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting mesh network status: {str(e)}")
