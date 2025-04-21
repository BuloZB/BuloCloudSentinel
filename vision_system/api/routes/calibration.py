"""
Calibration API routes for the Vision System.

This module provides endpoints for camera calibration and perspective mapping.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, File, UploadFile
from typing import List, Dict, Any, Optional
from pydantic import BaseModel

# Import local modules
from services.analysis_service import AnalysisService
from api.schemas.calibration import (
    CalibrationConfig,
    CalibrationResult,
    PerspectiveMap,
    ScaleFactor
)

router = APIRouter()

# Models
class CalibrationRequest(BaseModel):
    """Request model for calibrating a stream."""
    stream_id: str
    calibration_type: str  # intrinsic, extrinsic, perspective
    reference_points: Optional[List[Dict[str, float]]] = None
    reference_distances: Optional[List[Dict[str, float]]] = None
    parameters: Optional[Dict[str, Any]] = None

class PerspectiveMapRequest(BaseModel):
    """Request model for creating a perspective map."""
    stream_id: str
    image_points: List[Dict[str, float]]
    world_points: List[Dict[str, float]]
    parameters: Optional[Dict[str, Any]] = None

# Helper functions
def get_analysis_service(request: Request) -> AnalysisService:
    """Get the analysis service from the app state."""
    return request.app.state.analysis_service

# Routes
@router.get("/config")
async def get_calibration_config(
    stream_id: Optional[str] = None,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get the current calibration configuration."""
    try:
        config = await analysis_service.get_calibration_config(stream_id)
        return config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting calibration config: {str(e)}")

@router.post("/calibrate")
async def calibrate_stream(
    request: CalibrationRequest,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Calibrate a stream."""
    try:
        result = await analysis_service.calibrate_stream(
            stream_id=request.stream_id,
            calibration_type=request.calibration_type,
            reference_points=request.reference_points,
            reference_distances=request.reference_distances,
            parameters=request.parameters
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error calibrating stream: {str(e)}")

@router.post("/perspective-map")
async def create_perspective_map(
    request: PerspectiveMapRequest,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Create a perspective map for a stream."""
    try:
        map_result = await analysis_service.create_perspective_map(
            stream_id=request.stream_id,
            image_points=request.image_points,
            world_points=request.world_points,
            parameters=request.parameters
        )
        return map_result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating perspective map: {str(e)}")

@router.get("/perspective-map/{stream_id}")
async def get_perspective_map(
    stream_id: str,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get the perspective map for a stream."""
    try:
        map_result = await analysis_service.get_perspective_map(stream_id)
        if not map_result:
            raise HTTPException(status_code=404, detail=f"Perspective map for stream {stream_id} not found")
        return map_result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting perspective map: {str(e)}")

@router.delete("/perspective-map/{stream_id}")
async def delete_perspective_map(
    stream_id: str,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Delete the perspective map for a stream."""
    try:
        success = await analysis_service.delete_perspective_map(stream_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Perspective map for stream {stream_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting perspective map: {str(e)}")

@router.post("/scale-factor")
async def calculate_scale_factor(
    stream_id: str,
    reference_distance: float,
    point1_x: float,
    point1_y: float,
    point2_x: float,
    point2_y: float,
    unit: str = "meters",
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Calculate the scale factor for a stream."""
    try:
        scale_factor = await analysis_service.calculate_scale_factor(
            stream_id=stream_id,
            reference_distance=reference_distance,
            point1=(point1_x, point1_y),
            point2=(point2_x, point2_y),
            unit=unit
        )
        return scale_factor
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error calculating scale factor: {str(e)}")

@router.get("/scale-factor/{stream_id}")
async def get_scale_factor(
    stream_id: str,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get the scale factor for a stream."""
    try:
        scale_factor = await analysis_service.get_scale_factor(stream_id)
        if not scale_factor:
            raise HTTPException(status_code=404, detail=f"Scale factor for stream {stream_id} not found")
        return scale_factor
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting scale factor: {str(e)}")

@router.post("/calibration-pattern")
async def upload_calibration_pattern(
    stream_id: str,
    file: UploadFile = File(...),
    pattern_type: str = "checkerboard",
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Upload a calibration pattern image."""
    try:
        # Read image content
        content = await file.read()
        
        # Process calibration pattern
        result = await analysis_service.process_calibration_pattern(
            stream_id=stream_id,
            image_data=content,
            pattern_type=pattern_type
        )
        
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing calibration pattern: {str(e)}")

@router.get("/calibration-results/{stream_id}")
async def get_calibration_results(
    stream_id: str,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get calibration results for a stream."""
    try:
        results = await analysis_service.get_calibration_results(stream_id)
        if not results:
            raise HTTPException(status_code=404, detail=f"Calibration results for stream {stream_id} not found")
        return results
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting calibration results: {str(e)}")
