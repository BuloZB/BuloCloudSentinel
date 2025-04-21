"""
Analysis API routes for the Vision System.

This module provides endpoints for crowd and vehicle analysis.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, BackgroundTasks, File, UploadFile
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from datetime import datetime

# Import local modules
from services.analysis_service import AnalysisService
from api.schemas.analysis import (
    AnalysisConfig,
    AnalysisResult,
    CrowdAnalysisResult,
    VehicleAnalysisResult,
    FlowAnalysisResult,
    AnalysisJob,
    AnalysisJobStatus
)

router = APIRouter()

# Models
class AnalysisRequest(BaseModel):
    """Request model for starting an analysis job."""
    stream_id: str
    analysis_types: List[str]  # crowd, vehicle, flow, all
    region_of_interest: Optional[Dict[str, Any]] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    parameters: Optional[Dict[str, Any]] = None

class UpdateConfigRequest(BaseModel):
    """Request model for updating analysis configuration."""
    crowd_detection_enabled: Optional[bool] = None
    vehicle_detection_enabled: Optional[bool] = None
    flow_analysis_enabled: Optional[bool] = None
    crowd_density_threshold: Optional[float] = None
    vehicle_count_threshold: Optional[int] = None
    analysis_interval: Optional[float] = None  # seconds
    detection_confidence: Optional[float] = None

# Helper functions
def get_analysis_service(request: Request) -> AnalysisService:
    """Get the analysis service from the app state."""
    return request.app.state.analysis_service

# Routes
@router.get("/config")
async def get_analysis_config(
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get the current analysis configuration."""
    try:
        config = await analysis_service.get_config()
        return config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting analysis config: {str(e)}")

@router.put("/config")
async def update_analysis_config(
    request: UpdateConfigRequest,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Update the analysis configuration."""
    try:
        updated_config = await analysis_service.update_config(
            crowd_detection_enabled=request.crowd_detection_enabled,
            vehicle_detection_enabled=request.vehicle_detection_enabled,
            flow_analysis_enabled=request.flow_analysis_enabled,
            crowd_density_threshold=request.crowd_density_threshold,
            vehicle_count_threshold=request.vehicle_count_threshold,
            analysis_interval=request.analysis_interval,
            detection_confidence=request.detection_confidence
        )
        return updated_config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating analysis config: {str(e)}")

@router.post("/start")
async def start_analysis(
    request: AnalysisRequest,
    background_tasks: BackgroundTasks,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Start an analysis job."""
    try:
        job_id = await analysis_service.start_analysis_job(
            stream_id=request.stream_id,
            analysis_types=request.analysis_types,
            region_of_interest=request.region_of_interest,
            start_time=request.start_time,
            end_time=request.end_time,
            parameters=request.parameters
        )
        
        # Run analysis in background
        background_tasks.add_task(analysis_service.run_analysis_job, job_id)
        
        return {"job_id": job_id, "status": "started"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error starting analysis: {str(e)}")

@router.get("/jobs")
async def get_analysis_jobs(
    status: Optional[str] = None,
    limit: int = 10,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get all analysis jobs, optionally filtered by status."""
    try:
        jobs = await analysis_service.get_analysis_jobs(status, limit)
        return jobs
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting analysis jobs: {str(e)}")

@router.get("/jobs/{job_id}")
async def get_analysis_job(
    job_id: str,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get a specific analysis job by ID."""
    try:
        job = await analysis_service.get_analysis_job(job_id)
        if not job:
            raise HTTPException(status_code=404, detail=f"Analysis job {job_id} not found")
        return job
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting analysis job: {str(e)}")

@router.delete("/jobs/{job_id}")
async def cancel_analysis_job(
    job_id: str,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Cancel an analysis job."""
    try:
        success = await analysis_service.cancel_analysis_job(job_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Analysis job {job_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error canceling analysis job: {str(e)}")

@router.get("/results")
async def get_analysis_results(
    stream_id: Optional[str] = None,
    analysis_type: Optional[str] = None,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    limit: int = 10,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get analysis results, optionally filtered by stream, type, and time range."""
    try:
        results = await analysis_service.get_analysis_results(
            stream_id=stream_id,
            analysis_type=analysis_type,
            start_time=start_time,
            end_time=end_time,
            limit=limit
        )
        return results
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting analysis results: {str(e)}")

@router.get("/results/{result_id}")
async def get_analysis_result(
    result_id: str,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get a specific analysis result by ID."""
    try:
        result = await analysis_service.get_analysis_result(result_id)
        if not result:
            raise HTTPException(status_code=404, detail=f"Analysis result {result_id} not found")
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting analysis result: {str(e)}")

@router.post("/analyze-image")
async def analyze_image(
    file: UploadFile = File(...),
    analysis_types: Optional[List[str]] = None,
    parameters: Optional[Dict[str, Any]] = None,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Analyze a single image."""
    try:
        # Read image content
        content = await file.read()
        
        # Analyze image
        result = await analysis_service.analyze_image(
            image_data=content,
            analysis_types=analysis_types or ["crowd", "vehicle"],
            parameters=parameters
        )
        
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error analyzing image: {str(e)}")

@router.get("/crowd/density")
async def get_crowd_density(
    stream_id: str,
    region_of_interest: Optional[Dict[str, Any]] = None,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get current crowd density for a stream."""
    try:
        density = await analysis_service.get_current_crowd_density(
            stream_id=stream_id,
            region_of_interest=region_of_interest
        )
        return density
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting crowd density: {str(e)}")

@router.get("/vehicle/count")
async def get_vehicle_count(
    stream_id: str,
    vehicle_types: Optional[List[str]] = None,
    region_of_interest: Optional[Dict[str, Any]] = None,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get current vehicle count for a stream."""
    try:
        count = await analysis_service.get_current_vehicle_count(
            stream_id=stream_id,
            vehicle_types=vehicle_types,
            region_of_interest=region_of_interest
        )
        return count
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting vehicle count: {str(e)}")

@router.get("/flow/analysis")
async def get_flow_analysis(
    stream_id: str,
    duration: int = 60,  # seconds
    region_of_interest: Optional[Dict[str, Any]] = None,
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get flow analysis for a stream over a specified duration."""
    try:
        flow = await analysis_service.get_flow_analysis(
            stream_id=stream_id,
            duration=duration,
            region_of_interest=region_of_interest
        )
        return flow
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting flow analysis: {str(e)}")

@router.get("/statistics")
async def get_statistics(
    stream_id: Optional[str] = None,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    interval: str = "hour",  # hour, day, week, month
    analysis_service: AnalysisService = Depends(get_analysis_service)
):
    """Get statistical analysis over time."""
    try:
        stats = await analysis_service.get_statistics(
            stream_id=stream_id,
            start_time=start_time,
            end_time=end_time,
            interval=interval
        )
        return stats
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting statistics: {str(e)}")
