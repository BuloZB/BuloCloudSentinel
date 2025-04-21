"""
Missions API router for SentinelWeb.
"""

from fastapi import APIRouter, Depends, HTTPException, status, UploadFile, File, Form
from typing import List, Dict, Any, Optional
from datetime import datetime

from open_webui.internal.auth import get_current_user
from open_webui.models.users import User
from open_webui.sentinel_adapter.main import get_sentinel_adapter
from open_webui.sentinel_adapter.mission_adapter import Mission

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
    adapter = get_sentinel_adapter()
    missions = await adapter.get_mission_adapter().list_missions(status, drone_id)
    return missions

@router.get("/{mission_id}", response_model=Mission)
async def get_mission(mission_id: str, current_user: User = Depends(get_current_user)):
    """
    Get a specific mission.
    """
    adapter = get_sentinel_adapter()
    mission = await adapter.get_mission_adapter().get_mission(mission_id)
    
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
    adapter = get_sentinel_adapter()
    
    # Set created_by to current user
    mission_data["created_by"] = current_user.id
    
    mission = await adapter.get_mission_adapter().create_mission(mission_data)
    
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
    adapter = get_sentinel_adapter()
    
    # Check if mission exists
    existing_mission = await adapter.get_mission_adapter().get_mission(mission_id)
    if not existing_mission:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission with ID {mission_id} not found"
        )
    
    mission = await adapter.get_mission_adapter().update_mission(mission_id, mission_data)
    
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
    adapter = get_sentinel_adapter()
    
    # Check if mission exists
    existing_mission = await adapter.get_mission_adapter().get_mission(mission_id)
    if not existing_mission:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission with ID {mission_id} not found"
        )
    
    success = await adapter.get_mission_adapter().delete_mission(mission_id)
    
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
    adapter = get_sentinel_adapter()
    
    # Check if mission exists
    existing_mission = await adapter.get_mission_adapter().get_mission(mission_id)
    if not existing_mission:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission with ID {mission_id} not found"
        )
    
    result = await adapter.get_mission_adapter().execute_mission(mission_id)
    
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
    adapter = get_sentinel_adapter()
    
    # Check if mission exists
    existing_mission = await adapter.get_mission_adapter().get_mission(mission_id)
    if not existing_mission:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission with ID {mission_id} not found"
        )
    
    result = await adapter.get_mission_adapter().simulate_mission(mission_id)
    
    if "error" in result:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=result["error"]
        )
    
    return result

@router.post("/import")
async def import_mission(
    file: UploadFile = File(...),
    name: str = Form(...),
    current_user: User = Depends(get_current_user)
):
    """
    Import a mission from a file.
    """
    adapter = get_sentinel_adapter()
    
    # Check file extension
    if not file.filename.endswith(('.kml', '.gpx')):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="File must be KML or GPX"
        )
    
    # Get file format
    file_format = file.filename.split('.')[-1].lower()
    
    # Read file content
    file_content = await file.read()
    
    mission = await adapter.get_mission_adapter().import_mission(file_content, file_format, name)
    
    if not mission:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to import mission"
        )
    
    return mission

@router.get("/{mission_id}/export/{format}")
async def export_mission(
    mission_id: str,
    format: str,
    current_user: User = Depends(get_current_user)
):
    """
    Export a mission to a file.
    """
    adapter = get_sentinel_adapter()
    
    # Check if mission exists
    existing_mission = await adapter.get_mission_adapter().get_mission(mission_id)
    if not existing_mission:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Mission with ID {mission_id} not found"
        )
    
    # Check format
    if format.lower() not in ('kml', 'gpx'):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Format must be KML or GPX"
        )
    
    file_content = await adapter.get_mission_adapter().export_mission(mission_id, format.lower())
    
    if not file_content:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to export mission"
        )
    
    return file_content
