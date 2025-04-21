"""
Analytics API routes for the AI Analytics module.

This module provides endpoints for predictive analytics and historical data analysis.
"""

from fastapi import APIRouter, Depends, HTTPException, Request, BackgroundTasks
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from datetime import datetime

# Import local modules
from services.analytics import AnalyticsService
from api.schemas.analytics import (
    AnalyticsConfig,
    PredictionModel,
    AnalyticsReport,
    Anomaly
)

router = APIRouter()

# Models
class UpdateAnalyticsConfigRequest(BaseModel):
    """Request model for updating analytics configuration."""
    enabled: Optional[bool] = None
    anomaly_detection_enabled: Optional[bool] = None
    prediction_enabled: Optional[bool] = None
    data_retention_days: Optional[int] = None
    training_interval_hours: Optional[int] = None

class CreatePredictionModelRequest(BaseModel):
    """Request model for creating a prediction model."""
    name: str
    description: Optional[str] = None
    model_type: str  # time_series, classification, regression
    target_entity: str  # person, vehicle, face, license_plate, etc.
    target_property: str  # count, presence, direction, etc.
    parameters: Dict[str, Any]
    cameras: List[str]

class UpdatePredictionModelRequest(BaseModel):
    """Request model for updating a prediction model."""
    name: Optional[str] = None
    description: Optional[str] = None
    model_type: Optional[str] = None
    target_entity: Optional[str] = None
    target_property: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None
    cameras: Optional[List[str]] = None

class GenerateReportRequest(BaseModel):
    """Request model for generating an analytics report."""
    report_type: str  # daily, weekly, monthly, custom
    start_date: datetime
    end_date: Optional[datetime] = None
    cameras: Optional[List[str]] = None
    entities: Optional[List[str]] = None  # person, vehicle, face, etc.
    include_predictions: Optional[bool] = None
    include_anomalies: Optional[bool] = None
    format: Optional[str] = "json"  # json, csv, pdf

# Helper functions
def get_analytics_service(request: Request) -> AnalyticsService:
    """Get the analytics service from the app state."""
    return AnalyticsService(
        request.app.state.video_manager,
        request.app.state.event_publisher,
        request.app.state.config["analytics"]
    )

# Routes
@router.get("/config")
async def get_analytics_config(
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Get the current analytics configuration."""
    try:
        config = await analytics_service.get_config()
        return config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting analytics config: {str(e)}")

@router.put("/config")
async def update_analytics_config(
    request: UpdateAnalyticsConfigRequest,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Update the analytics configuration."""
    try:
        updated_config = await analytics_service.update_config(
            enabled=request.enabled,
            anomaly_detection_enabled=request.anomaly_detection_enabled,
            prediction_enabled=request.prediction_enabled,
            data_retention_days=request.data_retention_days,
            training_interval_hours=request.training_interval_hours
        )
        return updated_config
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating analytics config: {str(e)}")

@router.get("/models")
async def get_prediction_models(
    model_type: Optional[str] = None,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Get all prediction models, optionally filtered by type."""
    try:
        models = await analytics_service.get_models(model_type)
        return models
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting prediction models: {str(e)}")

@router.get("/models/{model_id}")
async def get_prediction_model(
    model_id: str,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Get a specific prediction model by ID."""
    try:
        model = await analytics_service.get_model(model_id)
        if not model:
            raise HTTPException(status_code=404, detail=f"Prediction model {model_id} not found")
        return model
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting prediction model: {str(e)}")

@router.post("/models")
async def create_prediction_model(
    request: CreatePredictionModelRequest,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Create a new prediction model."""
    try:
        model = await analytics_service.create_model(
            name=request.name,
            description=request.description,
            model_type=request.model_type,
            target_entity=request.target_entity,
            target_property=request.target_property,
            parameters=request.parameters,
            cameras=request.cameras
        )
        return model
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating prediction model: {str(e)}")

@router.put("/models/{model_id}")
async def update_prediction_model(
    model_id: str,
    request: UpdatePredictionModelRequest,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Update a prediction model."""
    try:
        model = await analytics_service.update_model(
            model_id=model_id,
            name=request.name,
            description=request.description,
            model_type=request.model_type,
            target_entity=request.target_entity,
            target_property=request.target_property,
            parameters=request.parameters,
            cameras=request.cameras
        )
        if not model:
            raise HTTPException(status_code=404, detail=f"Prediction model {model_id} not found")
        return model
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating prediction model: {str(e)}")

@router.delete("/models/{model_id}")
async def delete_prediction_model(
    model_id: str,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Delete a prediction model."""
    try:
        success = await analytics_service.delete_model(model_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Prediction model {model_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting prediction model: {str(e)}")

@router.post("/models/{model_id}/train")
async def train_prediction_model(
    model_id: str,
    background_tasks: BackgroundTasks,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Train a prediction model."""
    try:
        # Check if model exists
        model = await analytics_service.get_model(model_id)
        if not model:
            raise HTTPException(status_code=404, detail=f"Prediction model {model_id} not found")
        
        # Start training in background
        background_tasks.add_task(analytics_service.train_model, model_id)
        return {"message": f"Training started for model {model_id}"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error starting model training: {str(e)}")

@router.get("/predictions")
async def get_predictions(
    model_id: str,
    start_time: Optional[str] = None,
    end_time: Optional[str] = None,
    camera_id: Optional[str] = None,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Get predictions from a model."""
    try:
        predictions = await analytics_service.get_predictions(
            model_id=model_id,
            start_time=start_time,
            end_time=end_time,
            camera_id=camera_id
        )
        return predictions
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting predictions: {str(e)}")

@router.get("/anomalies")
async def get_anomalies(
    start_time: Optional[str] = None,
    end_time: Optional[str] = None,
    camera_id: Optional[str] = None,
    entity_type: Optional[str] = None,
    severity: Optional[str] = None,
    limit: int = 10,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Get detected anomalies."""
    try:
        anomalies = await analytics_service.get_anomalies(
            start_time=start_time,
            end_time=end_time,
            camera_id=camera_id,
            entity_type=entity_type,
            severity=severity,
            limit=limit
        )
        return anomalies
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting anomalies: {str(e)}")

@router.post("/reports")
async def generate_report(
    request: GenerateReportRequest,
    background_tasks: BackgroundTasks,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Generate an analytics report."""
    try:
        # Start report generation in background
        report_id = await analytics_service.start_report_generation(
            report_type=request.report_type,
            start_date=request.start_date,
            end_date=request.end_date,
            cameras=request.cameras,
            entities=request.entities,
            include_predictions=request.include_predictions,
            include_anomalies=request.include_anomalies,
            format=request.format
        )
        
        background_tasks.add_task(analytics_service.generate_report, report_id)
        return {"report_id": report_id, "status": "generating"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error starting report generation: {str(e)}")

@router.get("/reports/{report_id}")
async def get_report(
    report_id: str,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Get a generated report by ID."""
    try:
        report = await analytics_service.get_report(report_id)
        if not report:
            raise HTTPException(status_code=404, detail=f"Report {report_id} not found")
        return report
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting report: {str(e)}")

@router.get("/reports")
async def list_reports(
    report_type: Optional[str] = None,
    start_date: Optional[str] = None,
    end_date: Optional[str] = None,
    limit: int = 10,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """List generated reports."""
    try:
        reports = await analytics_service.list_reports(
            report_type=report_type,
            start_date=start_date,
            end_date=end_date,
            limit=limit
        )
        return reports
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error listing reports: {str(e)}")

@router.delete("/reports/{report_id}")
async def delete_report(
    report_id: str,
    analytics_service: AnalyticsService = Depends(get_analytics_service)
):
    """Delete a report."""
    try:
        success = await analytics_service.delete_report(report_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Report {report_id} not found")
        return {"success": True}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting report: {str(e)}")
