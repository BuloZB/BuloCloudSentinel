"""
SentinelWeb Backend - Missions Endpoints

This module provides endpoints for mission management.
"""

from typing import Any, List, Optional

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
import json

from fastapi import APIRouter, Depends, HTTPException, status, Request, File, UploadFile
from pydantic import BaseModel, UUID4

from backend.core.auth import get_current_user, User, has_role
from backend.services.sentinel_client import SentinelClient

router = APIRouter()

# Pydantic models
class Waypoint(BaseModel):
    """Waypoint model."""
    latitude: float
    longitude: float
    altitude: float
    heading: Optional[float] = None
    speed: Optional[float] = None
    action: Optional[str] = None
    hold_time: Optional[int] = None  # in seconds
    acceptance_radius: Optional[float] = None  # in meters

class MissionBase(BaseModel):
    """Base mission model."""
    name: str
    description: Optional[str] = None
    drone_id: Optional[str] = None
    mission_type: str
    waypoints: List[Waypoint]
    parameters: Optional[dict] = None

class MissionCreate(MissionBase):
    """Mission creation model."""
    pass

class MissionUpdate(BaseModel):
    """Mission update model."""
    name: Optional[str] = None
    description: Optional[str] = None
    drone_id: Optional[str] = None
    mission_type: Optional[str] = None
    waypoints: Optional[List[Waypoint]] = None
    parameters: Optional[dict] = None
    status: Optional[str] = None

class MissionResponse(MissionBase):
    """Mission response model."""
    id: str
    status: str
    created_at: Any
    updated_at: Any
    created_by: str

class MissionExecuteResponse(BaseModel):
    """Mission execution response model."""
    mission_id: str
    status: str
    message: str
    execution_id: Optional[str] = None

@router.get("/", response_model=List[MissionResponse])
async def get_missions(
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Get all missions.
    
    Args:
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        List of missions
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Get missions from BuloCloudSentinel
        missions = await sentinel_client.get_missions()
        return missions
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error getting missions from BuloCloudSentinel: {str(e)}"
        )

@router.get("/{mission_id}", response_model=MissionResponse)
async def get_mission(
    mission_id: str,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Get a mission by ID.
    
    Args:
        mission_id: Mission ID
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Mission information
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Get mission from BuloCloudSentinel
        mission = await sentinel_client.get_mission(mission_id)
        return mission
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error getting mission from BuloCloudSentinel: {str(e)}"
        )

@router.post("/", response_model=MissionResponse, dependencies=[Depends(has_role(["admin", "operator"]))])
async def create_mission(
    mission: MissionCreate,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Create a new mission.
    
    Args:
        mission: Mission data
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Created mission
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Create mission in BuloCloudSentinel
        created_mission = await sentinel_client.create_mission(mission.dict())
        return created_mission
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error creating mission in BuloCloudSentinel: {str(e)}"
        )

@router.put("/{mission_id}", response_model=MissionResponse, dependencies=[Depends(has_role(["admin", "operator"]))])
async def update_mission(
    mission_id: str,
    mission: MissionUpdate,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Update a mission.
    
    Args:
        mission_id: Mission ID
        mission: Mission data
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Updated mission
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Update mission in BuloCloudSentinel
        updated_mission = await sentinel_client.update_mission(
            mission_id=mission_id,
            mission_data=mission.dict(exclude_unset=True)
        )
        return updated_mission
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error updating mission in BuloCloudSentinel: {str(e)}"
        )

@router.delete("/{mission_id}", dependencies=[Depends(has_role(["admin", "operator"]))])
async def delete_mission(
    mission_id: str,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Delete a mission.
    
    Args:
        mission_id: Mission ID
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Deletion confirmation
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Delete mission in BuloCloudSentinel
        await sentinel_client.delete_mission(mission_id)
        return {"detail": "Mission deleted"}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error deleting mission in BuloCloudSentinel: {str(e)}"
        )

@router.post("/{mission_id}/execute", response_model=MissionExecuteResponse, dependencies=[Depends(has_role(["admin", "operator"]))])
async def execute_mission(
    mission_id: str,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Execute a mission.
    
    Args:
        mission_id: Mission ID
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Execution response
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Execute mission in BuloCloudSentinel
        response = await sentinel_client.execute_mission(mission_id)
        return response
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error executing mission in BuloCloudSentinel: {str(e)}"
        )

@router.post("/import/kml", response_model=MissionResponse, dependencies=[Depends(has_role(["admin", "operator"]))])
async def import_kml(
    file: UploadFile = File(...),
    name: str = None,
    description: str = None,
    drone_id: str = None,
    mission_type: str = "survey",
    request: Request = None,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Import a mission from a KML file.
    
    Args:
        file: KML file
        name: Mission name
        description: Mission description
        drone_id: Drone ID
        mission_type: Mission type
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Created mission
    """
    # Validate file
    if not file.filename.endswith(".kml"):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="File must be a KML file"
        )
    
    # Read file content
    content = await file.read()
    
    # Parse KML and extract waypoints
    # This is a placeholder - the actual implementation would depend on a KML parsing library
    try:
        # In a real implementation, you would use a library like pykml to parse the KML file
        # For now, we'll just create a dummy mission with some waypoints
        waypoints = [
            Waypoint(latitude=47.6062, longitude=-122.3321, altitude=50),
            Waypoint(latitude=47.6162, longitude=-122.3421, altitude=50),
            Waypoint(latitude=47.6262, longitude=-122.3521, altitude=50)
        ]
        
        # Create mission
        mission = MissionCreate(
            name=name or file.filename.replace(".kml", ""),
            description=description or "Imported from KML",
            drone_id=drone_id,
            mission_type=mission_type,
            waypoints=waypoints
        )
        
        # Create mission in BuloCloudSentinel
        sentinel_client: SentinelClient = request.app.state.sentinel_client
        created_mission = await sentinel_client.create_mission(mission.dict())
        
        return created_mission
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Error parsing KML file: {str(e)}"
        )

@router.post("/import/gpx", response_model=MissionResponse, dependencies=[Depends(has_role(["admin", "operator"]))])
async def import_gpx(
    file: UploadFile = File(...),
    name: str = None,
    description: str = None,
    drone_id: str = None,
    mission_type: str = "survey",
    request: Request = None,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Import a mission from a GPX file.
    
    Args:
        file: GPX file
        name: Mission name
        description: Mission description
        drone_id: Drone ID
        mission_type: Mission type
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        Created mission
    """
    # Validate file
    if not file.filename.endswith(".gpx"):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="File must be a GPX file"
        )
    
    # Read file content
    content = await file.read()
    
    # Parse GPX and extract waypoints
    # This is a placeholder - the actual implementation would depend on a GPX parsing library
    try:
        # In a real implementation, you would use a library like gpxpy to parse the GPX file
        # For now, we'll just create a dummy mission with some waypoints
        waypoints = [
            Waypoint(latitude=47.6062, longitude=-122.3321, altitude=50),
            Waypoint(latitude=47.6162, longitude=-122.3421, altitude=50),
            Waypoint(latitude=47.6262, longitude=-122.3521, altitude=50)
        ]
        
        # Create mission
        mission = MissionCreate(
            name=name or file.filename.replace(".gpx", ""),
            description=description or "Imported from GPX",
            drone_id=drone_id,
            mission_type=mission_type,
            waypoints=waypoints
        )
        
        # Create mission in BuloCloudSentinel
        sentinel_client: SentinelClient = request.app.state.sentinel_client
        created_mission = await sentinel_client.create_mission(mission.dict())
        
        return created_mission
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Error parsing GPX file: {str(e)}"
        )

@router.get("/{mission_id}/export/kml")
async def export_kml(
    mission_id: str,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Export a mission as a KML file.
    
    Args:
        mission_id: Mission ID
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        KML file
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Get mission from BuloCloudSentinel
        mission = await sentinel_client.get_mission(mission_id)
        
        # Generate KML
        # This is a placeholder - the actual implementation would depend on a KML generation library
        kml_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{mission.name}</name>
    <description>{mission.description or ""}</description>
    <Folder>
      <name>Waypoints</name>
"""
        
        for i, waypoint in enumerate(mission.waypoints):
            kml_content += f"""
      <Placemark>
        <name>Waypoint {i+1}</name>
        <Point>
          <coordinates>{waypoint.longitude},{waypoint.latitude},{waypoint.altitude}</coordinates>
        </Point>
      </Placemark>
"""
        
        kml_content += """
    </Folder>
  </Document>
</kml>
"""
        
        # Return KML file
        return {
            "content": kml_content,
            "filename": f"{mission.name.replace(' ', '_')}.kml",
            "content_type": "application/vnd.google-earth.kml+xml"
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error exporting mission to KML: {str(e)}"
        )

@router.get("/{mission_id}/export/gpx")
async def export_gpx(
    mission_id: str,
    request: Request,
    current_user: User = Depends(get_current_user)
) -> Any:
    """
    Export a mission as a GPX file.
    
    Args:
        mission_id: Mission ID
        request: FastAPI request
        current_user: Current authenticated user
        
    Returns:
        GPX file
    """
    # Get sentinel client from app state
    sentinel_client: SentinelClient = request.app.state.sentinel_client
    
    try:
        # Get mission from BuloCloudSentinel
        mission = await sentinel_client.get_mission(mission_id)
        
        # Generate GPX
        # This is a placeholder - the actual implementation would depend on a GPX generation library
        gpx_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="SentinelWeb" xmlns="http://www.topografix.com/GPX/1/1">
  <metadata>
    <name>{mission.name}</name>
    <desc>{mission.description or ""}</desc>
    <time>{mission.created_at}</time>
  </metadata>
  <trk>
    <name>{mission.name}</name>
    <trkseg>
"""
        
        for waypoint in mission.waypoints:
            gpx_content += f"""
      <trkpt lat="{waypoint.latitude}" lon="{waypoint.longitude}">
        <ele>{waypoint.altitude}</ele>
      </trkpt>
"""
        
        gpx_content += """
    </trkseg>
  </trk>
</gpx>
"""
        
        # Return GPX file
        return {
            "content": gpx_content,
            "filename": f"{mission.name.replace(' ', '_')}.gpx",
            "content_type": "application/gpx+xml"
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Error exporting mission to GPX: {str(e)}"
        )
