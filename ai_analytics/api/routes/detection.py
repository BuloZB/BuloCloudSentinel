"""
Detection API routes for the AI Analytics module.

This module provides endpoints for object detection configuration and management.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, BackgroundTasks
from typing import List, Dict, Any, Optional
from pydantic import BaseModel

# Import local modules
from services.detection import DetectionService
from api.schemas.detection import (
    DetectionConfig,
    DetectionResult,
    DetectionZone,
    ObjectClass
)

router = APIRouter()

# Models
class UpdateDetectionConfigRequest(BaseModel):
    """Request model for updating detection configuration."""
    model_name: Optional[str] = None
    confidence_threshold: Optional[float] = None
    enabled_classes: Optional[List[str]] = None
    zones: Optional[List[DetectionZone]] = None

class CreateDetectionZoneRequest(BaseModel):
    """Request model for creating a detection zone."""
    name: str
    camera_id: str
    points: List[Dict[str, float]]
    enabled_classes: Optional[List[str]] = None
    min_confidence: Optional[float] = None

# Helper functions
def get_detection_service(request: Request) -> DetectionService:
    """Get the detection service from the app state."""
    return DetectionService(
        request.app.state.video_manager,
        request.app.state.event_publisher,
        request.app.state.config["detection"]
    )

# Routes
@router.get("/config")
async def get_detection_config(
    camera_id: Optional[str] = None,
    detection_service: DetectionService = Depends(get_detection_service)
):
    """Get the current detection configuration."""
    try:
        config = await detection_service.get_config(camera_id)
        return config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting detection config: {str(e)}")

@router.put("/config")
async def update_detection_config(
    request: UpdateDetectionConfigRequest,
    camera_id: Optional[str] = None,
    detection_service: DetectionService = Depends(get_detection_service)
):
    """Update the detection configuration."""
    try:
        updated_config = await detection_service.update_config(
            camera_id=camera_id,
            model_name=request.model_name,
            confidence_threshold=request.confidence_threshold,
            enabled_classes=request.enabled_classes,
            zones=request.zones
        )
        return updated_config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating detection config: {str(e)}")

@router.get("/classes")
async def get_available_object_classes(
    detection_service: DetectionService = Depends(get_detection_service)
):
    """Get all available object classes for detection."""
    try:
        classes = await detection_service.get_available_classes()
        return classes
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting object classes: {str(e)}")

@router.get("/zones")
async def get_detection_zones(
    camera_id: Optional[str] = None,
    detection_service: DetectionService = Depends(get_detection_service)
):
    """Get all detection zones for a camera or all cameras."""
    try:
        zones = await detection_service.get_zones(camera_id)
        return zones
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting detection zones: {str(e)}")

@router.post("/zones")
async def create_detection_zone(
    request: CreateDetectionZoneRequest,
    detection_service: DetectionService = Depends(get_detection_service)
):
    """Create a new detection zone."""
    try:
        zone = await detection_service.create_zone(
            name=request.name,
            camera_id=request.camera_id,
            points=request.points,
            enabled_classes=request.enabled_classes,
            min_confidence=request.min_confidence
        )
        return zone
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating detection zone: {str(e)}")

@router.delete("/zones/{zone_id}")
async def delete_detection_zone(
    zone_id: str,
    detection_service: DetectionService = Depends(get_detection_service)
):
    """Delete a detection zone."""
    try:
        success = await detection_service.delete_zone(zone_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Zone {zone_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting detection zone: {str(e)}")

@router.get("/results")
async def get_recent_detection_results(
    camera_id: Optional[str] = None,
    limit: int = 10,
    detection_service: DetectionService = Depends(get_detection_service)
):
    """Get recent detection results."""
    try:
        results = await detection_service.get_recent_results(camera_id, limit)
        return results
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting detection results: {str(e)}")

@router.post("/test")
async def test_detection(
    camera_id: str,
    background_tasks: BackgroundTasks,
    detection_service: DetectionService = Depends(get_detection_service)
):
    """Run a test detection on a single frame from the camera."""
    try:
        # Run detection in background to avoid blocking
        background_tasks.add_task(detection_service.run_test_detection, camera_id)
        return {"message": f"Test detection started for camera {camera_id}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error starting test detection: {str(e)}")
