"""
Visualization API routes for the Vision System.

This module provides endpoints for visualization of analysis results.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, Response
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from datetime import datetime

# Import local modules
from services.analysis_service import AnalysisService
from processors.heat_mapper import HeatMapper
from api.schemas.visualization import (
    HeatMapConfig,
    VisualizationConfig,
    VisualizationResult
)

router = APIRouter()

# Models
class HeatMapRequest(BaseModel):
    """Request model for generating a heat map."""
    stream_id: str
    analysis_type: str  # crowd, vehicle, flow
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    resolution: Optional[Dict[str, int]] = None
    color_map: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None

class FlowMapRequest(BaseModel):
    """Request model for generating a flow map."""
    stream_id: str
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    resolution: Optional[Dict[str, int]] = None
    min_track_length: Optional[int] = None
    parameters: Optional[Dict[str, Any]] = None

class OverlayRequest(BaseModel):
    """Request model for generating an overlay visualization."""
    stream_id: str
    analysis_types: List[str]  # crowd, vehicle, flow
    show_detections: bool = True
    show_tracks: bool = False
    show_density: bool = False
    show_counts: bool = True
    parameters: Optional[Dict[str, Any]] = None

# Helper functions
def get_analysis_service(request: Request) -> AnalysisService:
    """Get the analysis service from the app state."""
    return request.app.state.analysis_service

def get_heat_mapper(request: Request) -> HeatMapper:
    """Get the heat mapper from the app state."""
    return request.app.state.analysis_service.heat_mapper

# Routes
@router.post("/heat-map")
async def generate_heat_map(
    request: HeatMapRequest,
    heat_mapper: HeatMapper = Depends(get_heat_mapper),
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Generate a heat map visualization."""
    try:
        # Get analysis results
        results = await analysis_service.get_analysis_results(
            stream_id=request.stream_id,
            analysis_type=request.analysis_type,
            start_time=request.start_time,
            end_time=request.end_time
        )
        
        if not results:
            raise HTTPException(status_code=404, detail=f"No analysis results found for the specified parameters")
        
        # Generate heat map
        heat_map = await heat_mapper.generate_heat_map(
            results=results,
            analysis_type=request.analysis_type,
            resolution=request.resolution,
            color_map=request.color_map,
            parameters=request.parameters
        )
        
        # Return heat map as image
        return Response(content=heat_map, media_type="image/png")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating heat map: {str(e)}")

@router.post("/flow-map")
async def generate_flow_map(
    request: FlowMapRequest,
    heat_mapper: HeatMapper = Depends(get_heat_mapper),
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Generate a flow map visualization."""
    try:
        # Get flow analysis results
        results = await analysis_service.get_analysis_results(
            stream_id=request.stream_id,
            analysis_type="flow",
            start_time=request.start_time,
            end_time=request.end_time
        )
        
        if not results:
            raise HTTPException(status_code=404, detail=f"No flow analysis results found for the specified parameters")
        
        # Generate flow map
        flow_map = await heat_mapper.generate_flow_map(
            results=results,
            resolution=request.resolution,
            min_track_length=request.min_track_length,
            parameters=request.parameters
        )
        
        # Return flow map as image
        return Response(content=flow_map, media_type="image/png")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating flow map: {str(e)}")

@router.post("/overlay")
async def generate_overlay(
    request: OverlayRequest,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Generate an overlay visualization on the current stream frame."""
    try:
        # Generate overlay
        overlay = await analysis_service.generate_overlay(
            stream_id=request.stream_id,
            analysis_types=request.analysis_types,
            show_detections=request.show_detections,
            show_tracks=request.show_tracks,
            show_density=request.show_density,
            show_counts=request.show_counts,
            parameters=request.parameters
        )
        
        # Return overlay as image
        return Response(content=overlay, media_type="image/jpeg")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating overlay: {str(e)}")

@router.get("/config")
async def get_visualization_config(
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get the current visualization configuration."""
    try:
        config = await analysis_service.get_visualization_config()
        return config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting visualization config: {str(e)}")

@router.put("/config")
async def update_visualization_config(
    config: VisualizationConfig,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Update the visualization configuration."""
    try:
        updated_config = await analysis_service.update_visualization_config(config)
        return updated_config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating visualization config: {str(e)}")

@router.get("/color-maps")
async def get_available_color_maps(
    heat_mapper: HeatMapper = Depends(get_heat_mapper)
):
    """Get all available color maps for heat map visualization."""
    try:
        color_maps = await heat_mapper.get_available_color_maps()
        return color_maps
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting color maps: {str(e)}")

@router.post("/time-lapse")
async def generate_time_lapse(
    stream_id: str,
    analysis_type: str,
    start_time: datetime,
    end_time: datetime,
    interval: int = 60,  # seconds
    fps: int = 10,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Generate a time-lapse video of analysis results."""
    try:
        # Generate time-lapse
        video_path = await analysis_service.generate_time_lapse(
            stream_id=stream_id,
            analysis_type=analysis_type,
            start_time=start_time,
            end_time=end_time,
            interval=interval,
            fps=fps
        )
        
        # Return path to video file
        return {"video_path": video_path}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating time-lapse: {str(e)}")

@router.get("/export/{result_id}")
async def export_visualization(
    result_id: str,
    format: str = "png",  # png, jpg, svg, pdf
    width: Optional[int] = None,
    height: Optional[int] = None,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Export a visualization result in various formats."""
    try:
        # Get visualization result
        result = await analysis_service.get_visualization_result(result_id)
        if not result:
            raise HTTPException(status_code=404, detail=f"Visualization result {result_id} not found")
        
        # Export visualization
        export_data = await analysis_service.export_visualization(
            result=result,
            format=format,
            width=width,
            height=height
        )
        
        # Set appropriate content type
        if format == "png":
            media_type = "image/png"
        elif format == "jpg":
            media_type = "image/jpeg"
        elif format == "svg":
            media_type = "image/svg+xml"
        elif format == "pdf":
            media_type = "application/pdf"
        else:
            media_type = "application/octet-stream"
        
        # Return exported visualization
        return Response(
            content=export_data,
            media_type=media_type,
            headers={"Content-Disposition": f"attachment; filename=visualization.{format}"}
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error exporting visualization: {str(e)}")
