"""
Behavior analysis API routes for the AI Analytics module.

This module provides endpoints for behavior analysis configuration and management.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, BackgroundTasks
from typing import List, Dict, Any, Optional
from pydantic import BaseModel

# Import local modules
from services.behavior import BehaviorService
from api.schemas.behavior import (
    BehaviorConfig,
    BehaviorPattern,
    BehaviorZone,
    BehaviorEvent
)

router = APIRouter()

# Models
class UpdateBehaviorConfigRequest(BaseModel):
    """Request model for updating behavior analysis configuration."""
    enabled: Optional[bool] = None
    loitering_threshold: Optional[int] = None  # seconds
    crowd_threshold: Optional[int] = None  # number of people
    running_speed_threshold: Optional[float] = None  # m/s
    direction_change_threshold: Optional[float] = None  # degrees

class CreateBehaviorPatternRequest(BaseModel):
    """Request model for creating a behavior pattern."""
    name: str
    description: Optional[str] = None
    pattern_type: str  # loitering, running, direction_change, crowd, etc.
    parameters: Dict[str, Any]
    alert_level: Optional[str] = None  # info, warning, critical

class UpdateBehaviorPatternRequest(BaseModel):
    """Request model for updating a behavior pattern."""
    name: Optional[str] = None
    description: Optional[str] = None
    pattern_type: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None
    alert_level: Optional[str] = None

class CreateBehaviorZoneRequest(BaseModel):
    """Request model for creating a behavior zone."""
    name: str
    camera_id: str
    points: List[Dict[str, float]]
    enabled_patterns: List[str]
    description: Optional[str] = None

# Helper functions
def get_behavior_service(request: Request) -> BehaviorService:
    """Get the behavior service from the app state."""
    return BehaviorService(
        request.app.state.video_manager,
        request.app.state.event_publisher,
        request.app.state.config["behavior"]
    )

# Routes
@router.get("/config")
async def get_behavior_config(
    camera_id: Optional[str] = None,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Get the current behavior analysis configuration."""
    try:
        config = await behavior_service.get_config(camera_id)
        return config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting behavior config: {str(e)}")

@router.put("/config")
async def update_behavior_config(
    request: UpdateBehaviorConfigRequest,
    camera_id: Optional[str] = None,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Update the behavior analysis configuration."""
    try:
        updated_config = await behavior_service.update_config(
            camera_id=camera_id,
            enabled=request.enabled,
            loitering_threshold=request.loitering_threshold,
            crowd_threshold=request.crowd_threshold,
            running_speed_threshold=request.running_speed_threshold,
            direction_change_threshold=request.direction_change_threshold
        )
        return updated_config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating behavior config: {str(e)}")

@router.get("/patterns")
async def get_behavior_patterns(
    pattern_type: Optional[str] = None,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Get all behavior patterns, optionally filtered by type."""
    try:
        patterns = await behavior_service.get_patterns(pattern_type)
        return patterns
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting behavior patterns: {str(e)}")

@router.get("/patterns/{pattern_id}")
async def get_behavior_pattern(
    pattern_id: str,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Get a specific behavior pattern by ID."""
    try:
        pattern = await behavior_service.get_pattern(pattern_id)
        if not pattern:
            raise HTTPException(status_code=404, detail=f"Behavior pattern {pattern_id} not found")
        return pattern
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting behavior pattern: {str(e)}")

@router.post("/patterns")
async def create_behavior_pattern(
    request: CreateBehaviorPatternRequest,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Create a new behavior pattern."""
    try:
        pattern = await behavior_service.create_pattern(
            name=request.name,
            description=request.description,
            pattern_type=request.pattern_type,
            parameters=request.parameters,
            alert_level=request.alert_level
        )
        return pattern
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating behavior pattern: {str(e)}")

@router.put("/patterns/{pattern_id}")
async def update_behavior_pattern(
    pattern_id: str,
    request: UpdateBehaviorPatternRequest,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Update a behavior pattern."""
    try:
        pattern = await behavior_service.update_pattern(
            pattern_id=pattern_id,
            name=request.name,
            description=request.description,
            pattern_type=request.pattern_type,
            parameters=request.parameters,
            alert_level=request.alert_level
        )
        if not pattern:
            raise HTTPException(status_code=404, detail=f"Behavior pattern {pattern_id} not found")
        return pattern
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating behavior pattern: {str(e)}")

@router.delete("/patterns/{pattern_id}")
async def delete_behavior_pattern(
    pattern_id: str,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Delete a behavior pattern."""
    try:
        success = await behavior_service.delete_pattern(pattern_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Behavior pattern {pattern_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting behavior pattern: {str(e)}")

@router.get("/zones")
async def get_behavior_zones(
    camera_id: Optional[str] = None,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Get all behavior zones for a camera or all cameras."""
    try:
        zones = await behavior_service.get_zones(camera_id)
        return zones
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting behavior zones: {str(e)}")

@router.get("/zones/{zone_id}")
async def get_behavior_zone(
    zone_id: str,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Get a specific behavior zone by ID."""
    try:
        zone = await behavior_service.get_zone(zone_id)
        if not zone:
            raise HTTPException(status_code=404, detail=f"Behavior zone {zone_id} not found")
        return zone
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting behavior zone: {str(e)}")

@router.post("/zones")
async def create_behavior_zone(
    request: CreateBehaviorZoneRequest,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Create a new behavior zone."""
    try:
        zone = await behavior_service.create_zone(
            name=request.name,
            camera_id=request.camera_id,
            points=request.points,
            enabled_patterns=request.enabled_patterns,
            description=request.description
        )
        return zone
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating behavior zone: {str(e)}")

@router.delete("/zones/{zone_id}")
async def delete_behavior_zone(
    zone_id: str,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Delete a behavior zone."""
    try:
        success = await behavior_service.delete_zone(zone_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Behavior zone {zone_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting behavior zone: {str(e)}")

@router.get("/events")
async def get_behavior_events(
    camera_id: Optional[str] = None,
    pattern_type: Optional[str] = None,
    start_time: Optional[str] = None,
    end_time: Optional[str] = None,
    limit: int = 10,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Get behavior events, optionally filtered by camera, pattern type, and time range."""
    try:
        events = await behavior_service.get_events(
            camera_id=camera_id,
            pattern_type=pattern_type,
            start_time=start_time,
            end_time=end_time,
            limit=limit
        )
        return events
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting behavior events: {str(e)}")

@router.post("/test")
async def test_behavior_analysis(
    camera_id: str,
    pattern_id: Optional[str] = None,
    background_tasks: BackgroundTasks,
    behavior_service: BehaviorService = Depends(get_behavior_service)
):
    """Run a test behavior analysis on a single frame from the camera."""
    try:
        # Run behavior analysis in background to avoid blocking
        background_tasks.add_task(behavior_service.run_test_analysis, camera_id, pattern_id)
        return {"message": f"Test behavior analysis started for camera {camera_id}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error starting test behavior analysis: {str(e)}")
