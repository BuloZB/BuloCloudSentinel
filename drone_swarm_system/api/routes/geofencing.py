"""
Geofencing API routes for the Drone Swarm System.

This module provides endpoints for geofencing and no-fly zone management.
"""

from fastapi import APIRouter, Depends, HTTPException, Request
from typing import List, Dict, Any, Optional
from pydantic import BaseModel

# Import local modules
from planning.geofencing import GeofencingManager
from api.schemas.geofencing import (
    GeofenceZone,
    GeofenceZoneCreate,
    GeofenceZoneUpdate,
    GeofenceType
)

router = APIRouter()

# Models
class CheckPositionRequest(BaseModel):
    """Request model for checking if a position is within a geofence zone."""
    latitude: float
    longitude: float
    altitude: Optional[float] = None

# Helper functions
def get_geofencing_manager(request: Request) -> GeofencingManager:
    """Get the geofencing manager from the app state."""
    return request.app.state.mission_planner.geofencing_manager

# Routes
@router.get("/zones")
async def get_all_zones(
    zone_type: Optional[str] = None,
    geofencing_manager: GeofencingManager = Depends(get_geofencing_manager)
):
    """Get all geofence zones, optionally filtered by type."""
    try:
        zones = await geofencing_manager.get_all_zones(zone_type)
        return zones
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting geofence zones: {str(e)}")

@router.get("/zones/{zone_id}")
async def get_zone(
    zone_id: str,
    geofencing_manager: GeofencingManager = Depends(get_geofencing_manager)
):
    """Get a specific geofence zone by ID."""
    try:
        zone = await geofencing_manager.get_zone(zone_id)
        if not zone:
            raise HTTPException(status_code=404, detail=f"Geofence zone {zone_id} not found")
        return zone
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting geofence zone: {str(e)}")

@router.post("/zones")
async def create_zone(
    zone: GeofenceZoneCreate,
    geofencing_manager: GeofencingManager = Depends(get_geofencing_manager)
):
    """Create a new geofence zone."""
    try:
        new_zone = await geofencing_manager.create_zone(zone)
        return new_zone
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating geofence zone: {str(e)}")

@router.put("/zones/{zone_id}")
async def update_zone(
    zone_id: str,
    zone: GeofenceZoneUpdate,
    geofencing_manager: GeofencingManager = Depends(get_geofencing_manager)
):
    """Update a geofence zone."""
    try:
        updated_zone = await geofencing_manager.update_zone(zone_id, zone)
        if not updated_zone:
            raise HTTPException(status_code=404, detail=f"Geofence zone {zone_id} not found")
        return updated_zone
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating geofence zone: {str(e)}")

@router.delete("/zones/{zone_id}")
async def delete_zone(
    zone_id: str,
    geofencing_manager: GeofencingManager = Depends(get_geofencing_manager)
):
    """Delete a geofence zone."""
    try:
        success = await geofencing_manager.delete_zone(zone_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Geofence zone {zone_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting geofence zone: {str(e)}")

@router.post("/check-position")
async def check_position(
    request: CheckPositionRequest,
    geofencing_manager: GeofencingManager = Depends(get_geofencing_manager)
):
    """Check if a position is within any geofence zone."""
    try:
        result = await geofencing_manager.check_position(
            request.latitude,
            request.longitude,
            request.altitude
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error checking position: {str(e)}")

@router.get("/types")
async def get_geofence_types(
    geofencing_manager: GeofencingManager = Depends(get_geofencing_manager)
):
    """Get all available geofence types."""
    try:
        types = await geofencing_manager.get_geofence_types()
        return types
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting geofence types: {str(e)}")

@router.post("/import")
async def import_geofence_zones(
    file: UploadFile = File(...),
    format: str = "geojson",
    geofencing_manager: GeofencingManager = Depends(get_geofencing_manager)
):
    """Import geofence zones from a file."""
    try:
        content = await file.read()
        zones = await geofencing_manager.import_zones(content, format)
        return {"imported_zones": len(zones), "zones": zones}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error importing geofence zones: {str(e)}")

@router.get("/export")
async def export_geofence_zones(
    format: str = "geojson",
    zone_type: Optional[str] = None,
    geofencing_manager: GeofencingManager = Depends(get_geofencing_manager)
):
    """Export geofence zones to a file."""
    try:
        content = await geofencing_manager.export_zones(format, zone_type)
        
        # Create response with appropriate content type
        if format == "geojson":
            media_type = "application/geo+json"
        elif format == "kml":
            media_type = "application/vnd.google-earth.kml+xml"
        else:
            media_type = "application/octet-stream"
        
        return Response(
            content=content,
            media_type=media_type,
            headers={"Content-Disposition": f"attachment; filename=geofence_zones.{format}"}
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error exporting geofence zones: {str(e)}")

@router.get("/zones/{zone_id}/check-mission/{mission_id}")
async def check_mission_against_zone(
    zone_id: str,
    mission_id: str,
    geofencing_manager: GeofencingManager = Depends(get_geofencing_manager)
):
    """Check if a mission violates a specific geofence zone."""
    try:
        result = await geofencing_manager.check_mission_against_zone(zone_id, mission_id)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error checking mission against zone: {str(e)}")
