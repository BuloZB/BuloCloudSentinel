"""
API routes for multimodal detection.

This module provides FastAPI routes for multimodal detection.
"""

import logging
from typing import List, Dict, Any, Optional
from fastapi import APIRouter, Depends, HTTPException, Query, Path, Body

from services.multimodal_detection import MultimodalDetectionService
from api.schemas.multimodal import (
    StartDetectionRequest,
    StartDetectionResponse,
    StopDetectionRequest,
    StopDetectionResponse,
    GetDetectionStatusResponse,
    GetActiveTasksResponse,
    MultimodalDetectionConfig,
    MultimodalDetectionZone
)
from api.dependencies import get_multimodal_detection_service

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create router
router = APIRouter(
    prefix="/multimodal",
    tags=["multimodal"],
    responses={404: {"description": "Not found"}}
)


@router.post("/detection/start", response_model=StartDetectionResponse)
async def start_detection(
    request: StartDetectionRequest,
    detection_service: MultimodalDetectionService = Depends(get_multimodal_detection_service)
):
    """
    Start multimodal detection on specified streams.
    
    Args:
        request: Start detection request
        detection_service: Multimodal detection service
        
    Returns:
        Start detection response
    """
    try:
        # Start detection
        task_id = await detection_service.start_detection(
            stream_ids=request.stream_ids,
            config=request.config
        )
        
        # Return response
        return StartDetectionResponse(
            task_id=task_id,
            status="started"
        )
    
    except Exception as e:
        logger.error(f"Error starting multimodal detection: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/detection/stop", response_model=StopDetectionResponse)
async def stop_detection(
    request: StopDetectionRequest,
    detection_service: MultimodalDetectionService = Depends(get_multimodal_detection_service)
):
    """
    Stop a multimodal detection task.
    
    Args:
        request: Stop detection request
        detection_service: Multimodal detection service
        
    Returns:
        Stop detection response
    """
    try:
        # Stop detection
        success = await detection_service.stop_detection(request.task_id)
        
        # Return response
        return StopDetectionResponse(
            task_id=request.task_id,
            status="stopped" if success else "not_found"
        )
    
    except Exception as e:
        logger.error(f"Error stopping multimodal detection: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/detection/status/{task_id}", response_model=GetDetectionStatusResponse)
async def get_detection_status(
    task_id: str = Path(..., description="Task ID"),
    detection_service: MultimodalDetectionService = Depends(get_multimodal_detection_service)
):
    """
    Get the status of a multimodal detection task.
    
    Args:
        task_id: Task ID
        detection_service: Multimodal detection service
        
    Returns:
        Detection status response
    """
    try:
        # Get status
        status = await detection_service.get_detection_status(task_id)
        
        # Return response
        return GetDetectionStatusResponse(**status)
    
    except Exception as e:
        logger.error(f"Error getting detection status: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/detection/tasks", response_model=GetActiveTasksResponse)
async def get_active_tasks(
    detection_service: MultimodalDetectionService = Depends(get_multimodal_detection_service)
):
    """
    Get all active multimodal detection tasks.
    
    Args:
        detection_service: Multimodal detection service
        
    Returns:
        Active tasks response
    """
    try:
        # Get active tasks
        tasks = await detection_service.get_active_tasks()
        
        # Return response
        return GetActiveTasksResponse(tasks=tasks)
    
    except Exception as e:
        logger.error(f"Error getting active tasks: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/detection/zones", response_model=Dict[str, MultimodalDetectionZone])
async def create_detection_zones(
    zones: List[MultimodalDetectionZone],
    detection_service: MultimodalDetectionService = Depends(get_multimodal_detection_service)
):
    """
    Create detection zones.
    
    Args:
        zones: List of detection zones
        detection_service: Multimodal detection service
        
    Returns:
        Dictionary mapping zone IDs to zone objects
    """
    try:
        # Create zones
        for zone in zones:
            detection_service.detection_zones[zone.id] = zone
        
        # Return zones
        return detection_service.detection_zones
    
    except Exception as e:
        logger.error(f"Error creating detection zones: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/detection/zones", response_model=Dict[str, MultimodalDetectionZone])
async def get_detection_zones(
    detection_service: MultimodalDetectionService = Depends(get_multimodal_detection_service)
):
    """
    Get all detection zones.
    
    Args:
        detection_service: Multimodal detection service
        
    Returns:
        Dictionary mapping zone IDs to zone objects
    """
    try:
        # Return zones
        return detection_service.detection_zones
    
    except Exception as e:
        logger.error(f"Error getting detection zones: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/detection/zones/{zone_id}", response_model=Dict[str, MultimodalDetectionZone])
async def delete_detection_zone(
    zone_id: str = Path(..., description="Zone ID"),
    detection_service: MultimodalDetectionService = Depends(get_multimodal_detection_service)
):
    """
    Delete a detection zone.
    
    Args:
        zone_id: Zone ID
        detection_service: Multimodal detection service
        
    Returns:
        Dictionary mapping zone IDs to zone objects
    """
    try:
        # Delete zone
        if zone_id in detection_service.detection_zones:
            del detection_service.detection_zones[zone_id]
        
        # Return zones
        return detection_service.detection_zones
    
    except Exception as e:
        logger.error(f"Error deleting detection zone: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/detection/config", response_model=MultimodalDetectionConfig)
async def get_default_config(
    detection_service: MultimodalDetectionService = Depends(get_multimodal_detection_service)
):
    """
    Get default detection configuration.
    
    Args:
        detection_service: Multimodal detection service
        
    Returns:
        Default detection configuration
    """
    try:
        # Get default config
        config = MultimodalDetectionConfig()
        
        # Return config
        return config
    
    except Exception as e:
        logger.error(f"Error getting default config: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))
