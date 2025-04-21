"""
Missions API router for SentinelWeb.
"""

from fastapi import APIRouter, Depends, HTTPException, status, UploadFile, File, Form
from typing import List, Dict, Any, Optional
from datetime import datetime

from sentinel_web.internal.auth import get_current_user
from sentinel_web.models.users import User
from sentinel_web.drone_adapter.adapter import get_drone_adapter
from sentinel_web.drone_adapter.mission_client import Mission

router = APIRouter()

@router.get("/", response_model=List[Mission])
async def list_missions(
    status: Optional[str] = None,
    drone_id: Optional[str] = None,
    current_user: User = Depends(get_current_user)
):
    """
    List all missions with optional filtering.
    """
    adapter = get_drone_adapter()
    missions = await adapter.get_mission_client().list_missions(status, drone_id)
    return missions

@router.get("/{mission_id}", response_model=Mission)
async def get_mission(mission_id: str, current_user: User = Depends(get_current_user)):
    """
    Get a specific mission.
    """
    adapter = get_drone_adapter()
    mission = await adapter.get_mission_client().get_mission(mission_id)
    
    if not mission:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission with ID {mission_id} not found"
        )
    
    return mission

@router.post("/", response_model=Mission)
async def create_mission(
    mission_data: Dict[str, Any],
    current_user: User = Depends(get_current_user)
):
    """
    Create a new mission.
    """
    adapter = get_drone_adapter()
    
    # Set created_by to current user
    mission_data["created_by"] = current_user.id
    
    mission = await adapter.get_mission_client().create_mission(mission_data)
    
    if not mission:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create mission"
        )
    
    return mission

@router.put("/{mission_id}", response_model=Mission)
async def update_mission(
    mission_id: str,
    mission_data: Dict[str, Any],
    current_user: User = Depends(get_current_user)
):
    """
    Update a mission.
    """
    adapter = get_drone_adapter()
    
    # Check if mission exists
    existing_mission = await adapter.get_mission_client().get_mission(mission_id)
    if not existing_mission:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission with ID {mission_id} not found"
        )
    
    mission = await adapter.get_mission_client().update_mission(mission_id, mission_data)
    
    if not mission:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update mission"
        )
    
    return mission

@router.delete("/{mission_id}")
async def delete_mission(mission_id: str, current_user: User = Depends(get_current_user)):
    """
    Delete a mission.
    """
    adapter = get_drone_adapter()
    
    # Check if mission exists
    existing_mission = await adapter.get_mission_client().get_mission(mission_id)
    if not existing_mission:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission with ID {mission_id} not found"
        )
    
    success = await adapter.get_mission_client().delete_mission(mission_id)
    
    if not success:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to delete mission"
        )
    
    return {"message": "Mission deleted successfully"}

@router.post("/{mission_id}/execute")
async def execute_mission(mission_id: str, current_user: User = Depends(get_current_user)):
    """
    Execute a mission.
    """
    adapter = get_drone_adapter()
    
    # Check if mission exists
    existing_mission = await adapter.get_mission_client().get_mission(mission_id)
    if not existing_mission:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission with ID {mission_id} not found"
        )
    
    result = await adapter.get_mission_client().execute_mission(mission_id)
    
    if "error" in result:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=result["error"]
        )
    
    return result

@router.post("/{mission_id}/simulate")
async def simulate_mission(mission_id: str, current_user: User = Depends(get_current_user)):
    """
    Simulate a mission.
    """
    adapter = get_drone_adapter()
    
    # Check if mission exists
    existing_mission = await adapter.get_mission_client().get_mission(mission_id)
    if not existing_mission:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission with ID {mission_id} not found"
        )
    
    result = await adapter.get_mission_client().simulate_mission(mission_id)
    
    if "error" in result:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=result["error"]
        )
    
    return result
