"""
API endpoints for geofencing.

This module provides FastAPI endpoints for managing geofence zones and
validating missions against them.
"""

from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from typing import List, Dict, Any, Optional

from backend.geofencing.models import (
    GeofenceZone, GeofenceZoneResponse, GeofenceProviderResponse,
    CustomGeofenceCreate, CustomGeofenceUpdate,
    MissionValidationRequest, MissionValidationResponse,
    WaypointValidationRequest, WaypointValidationResponse,
    GeofenceZoneType
)
from backend.geofencing.service import GeofencingService
from backend.api.dependencies import get_current_user

router = APIRouter(prefix="/geofence", tags=["Geofencing"])

# Global service instance
geofencing_service = None


# Initialize service
def initialize_service(service: GeofencingService):
    """Initialize the geofencing service."""
    global geofencing_service
    geofencing_service = service


# API endpoints
@router.get("/zones", response_model=List[GeofenceZoneResponse])
async def list_geofence_zones(
    latitude: Optional[float] = None,
    longitude: Optional[float] = None,
    radius: Optional[float] = None,
    zone_type: Optional[str] = None,
    current_user: str = Depends(get_current_user)
):
    """List geofence zones, optionally filtered by location and type."""
    if not geofencing_service:
        raise HTTPException(status_code=500, detail="Geofencing service not initialized")
    
    # Convert zone_type string to enum if provided
    zone_type_enum = None
    if zone_type:
        try:
            zone_type_enum = GeofenceZoneType(zone_type)
        except ValueError:
            raise HTTPException(status_code=400, detail=f"Invalid zone type: {zone_type}")
    
    try:
        zones = await geofencing_service.list_zones(
            latitude=latitude,
            longitude=longitude,
            radius=radius,
            zone_type=zone_type_enum
        )
        return [GeofenceZoneResponse(**zone.dict()) for zone in zones]
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error listing geofence zones: {str(e)}")


@router.get("/zones/{zone_id}", response_model=GeofenceZoneResponse)
async def get_geofence_zone(
    zone_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get details of a specific geofence zone."""
    if not geofencing_service:
        raise HTTPException(status_code=500, detail="Geofencing service not initialized")
    
    zone = await geofencing_service.get_zone(zone_id)
    if not zone:
        raise HTTPException(status_code=404, detail=f"Geofence zone {zone_id} not found")
    
    return GeofenceZoneResponse(**zone.dict())


@router.post("/zones/custom", response_model=GeofenceZoneResponse)
async def create_custom_geofence(
    zone: CustomGeofenceCreate,
    current_user: str = Depends(get_current_user)
):
    """Create a custom geofence zone."""
    if not geofencing_service:
        raise HTTPException(status_code=500, detail="Geofencing service not initialized")
    
    try:
        # Create zone
        geofence_zone = GeofenceZone(
            name=zone.name,
            description=zone.description,
            zone_type=GeofenceZoneType.CUSTOM,  # Will be set by service
            source="custom",  # Will be set by service
            restriction_level=zone.restriction_level,
            geometry_type=zone.geometry_type,
            geometry=zone.geometry,
            altitude_min=zone.altitude_min,
            altitude_max=zone.altitude_max,
            effective_from=zone.effective_from,
            effective_to=zone.effective_to,
            properties=zone.properties
        )
        
        created_zone = await geofencing_service.create_custom_zone(geofence_zone)
        return GeofenceZoneResponse(**created_zone.dict())
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating custom geofence: {str(e)}")


@router.put("/zones/custom/{zone_id}", response_model=GeofenceZoneResponse)
async def update_custom_geofence(
    zone_id: str,
    zone: CustomGeofenceUpdate,
    current_user: str = Depends(get_current_user)
):
    """Update a custom geofence zone."""
    if not geofencing_service:
        raise HTTPException(status_code=500, detail="Geofencing service not initialized")
    
    try:
        # Convert to dict and remove None values
        updates = {k: v for k, v in zone.dict().items() if v is not None}
        
        updated_zone = await geofencing_service.update_custom_zone(zone_id, updates)
        if not updated_zone:
            raise HTTPException(status_code=404, detail=f"Geofence zone {zone_id} not found")
        
        return GeofenceZoneResponse(**updated_zone.dict())
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating custom geofence: {str(e)}")


@router.delete("/zones/custom/{zone_id}")
async def delete_custom_geofence(
    zone_id: str,
    current_user: str = Depends(get_current_user)
):
    """Delete a custom geofence zone."""
    if not geofencing_service:
        raise HTTPException(status_code=500, detail="Geofencing service not initialized")
    
    try:
        success = await geofencing_service.delete_custom_zone(zone_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Geofence zone {zone_id} not found")
        
        return {"message": f"Geofence zone {zone_id} deleted"}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting custom geofence: {str(e)}")


@router.post("/validate-mission", response_model=MissionValidationResponse)
async def validate_mission(
    request: MissionValidationRequest,
    current_user: str = Depends(get_current_user)
):
    """Validate a mission plan against geofence zones."""
    if not geofencing_service:
        raise HTTPException(status_code=500, detail="Geofencing service not initialized")
    
    try:
        valid, violations = await geofencing_service.validate_mission(
            mission_id=request.mission_id,
            waypoints=request.waypoints
        )
        
        return MissionValidationResponse(
            valid=valid,
            violations=violations
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except NotImplementedError as e:
        raise HTTPException(status_code=501, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error validating mission: {str(e)}")


@router.post("/validate-waypoint", response_model=WaypointValidationResponse)
async def validate_waypoint(
    request: WaypointValidationRequest,
    current_user: str = Depends(get_current_user)
):
    """Validate a single waypoint against geofence zones."""
    if not geofencing_service:
        raise HTTPException(status_code=500, detail="Geofencing service not initialized")
    
    try:
        valid, violations = await geofencing_service.validate_waypoint(
            latitude=request.latitude,
            longitude=request.longitude,
            altitude=request.altitude
        )
        
        return WaypointValidationResponse(
            valid=valid,
            violations=violations
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error validating waypoint: {str(e)}")


@router.get("/providers", response_model=List[GeofenceProviderResponse])
async def list_geofence_providers(
    current_user: str = Depends(get_current_user)
):
    """List available geofence data providers."""
    if not geofencing_service:
        raise HTTPException(status_code=500, detail="Geofencing service not initialized")
    
    providers = await geofencing_service.list_providers()
    return [GeofenceProviderResponse(**provider.dict()) for provider in providers]


@router.post("/update-database")
async def update_geofence_database(
    background_tasks: BackgroundTasks,
    current_user: str = Depends(get_current_user)
):
    """Trigger an update of the geofence database from external providers."""
    if not geofencing_service:
        raise HTTPException(status_code=500, detail="Geofencing service not initialized")
    
    # Run update in background
    background_tasks.add_task(geofencing_service.update_database)
    
    return {"message": "Geofence database update started"}
