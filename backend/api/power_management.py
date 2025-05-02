"""
API endpoints for power management.

This module provides FastAPI endpoints for battery monitoring,
energy prediction, and power optimization.
"""

from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks

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
from typing import List, Dict, Any, Optional
from datetime import datetime

from backend.power_management.models import (
    BatteryMetrics, BatteryCurrentResponse, BatteryHistoryResponse,
    FleetBatteryResponse, BatteryThresholdsRequest, BatteryThresholdsResponse,
    EnergyPredictionRequest, EnergyPredictionResponse,
    OptimizeMissionRequest, OptimizeMissionResponse,
    BatteryHealthRequest, BatteryHealthResponse,
    EnergyPredictionModel
)
from backend.power_management.battery_service import BatteryMonitoringService
from backend.power_management.energy_prediction import EnergyPredictionService
from backend.api.dependencies import get_current_user

router = APIRouter(prefix="/power", tags=["Power Management"])

# Global service instances
battery_service = None
energy_prediction_service = None


# Initialize services

def validate_request_data(request_data: dict, schema: dict) -> dict:
    """
    Validate request data against a schema.

    Args:
        request_data: Request data to validate
        schema: Validation schema

    Returns:
        Validated request data
    """
    return request_validator.validate_request(request_data, schema)

def initialize_services(
    battery_monitoring_service: BatteryMonitoringService,
    energy_pred_service: EnergyPredictionService
):
    """Initialize power management services."""
    global battery_service, energy_prediction_service
    battery_service = battery_monitoring_service
    energy_prediction_service = energy_pred_service


# API endpoints
@router.get("/battery/{drone_id}/current", response_model=BatteryCurrentResponse)
async def get_battery_current(
    drone_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get current battery status for a specific drone."""
    if not battery_service:
        raise HTTPException(status_code=500, detail="Battery monitoring service not initialized")
    
    metrics = await battery_service.get_battery_metrics(drone_id)
    if not metrics:
        raise HTTPException(status_code=404, detail=f"Battery metrics for drone {drone_id} not found")
    
    return BatteryCurrentResponse(**metrics.dict())


@router.get("/battery/{drone_id}/history", response_model=BatteryHistoryResponse)
async def get_battery_history(
    drone_id: str,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    interval: str = "1m",
    current_user: str = Depends(get_current_user)
):
    """Get historical battery data for a specific drone."""
    if not battery_service:
        raise HTTPException(status_code=500, detail="Battery monitoring service not initialized")
    
    try:
        metrics = await battery_service.get_battery_history(
            drone_id=drone_id,
            start_time=start_time,
            end_time=end_time,
            interval=interval
        )
        
        # Convert to response format
        history_items = [
            {
                "timestamp": m.timestamp,
                "voltage": m.voltage,
                "current": m.current,
                "capacity_percent": m.capacity_percent,
                "temperature": m.temperature,
                "discharge_rate": m.discharge_rate,
                "status": m.status
            }
            for m in metrics
        ]
        
        return BatteryHistoryResponse(
            drone_id=drone_id,
            start_time=start_time or (datetime.utcnow() - metrics[0].timestamp if metrics else datetime.utcnow()),
            end_time=end_time or datetime.utcnow(),
            interval=interval,
            data=history_items
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting battery history: {str(e)}")


@router.get("/battery/fleet/{fleet_id}", response_model=FleetBatteryResponse)
async def get_fleet_battery(
    fleet_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get battery status for all drones in a fleet."""
    if not battery_service:
        raise HTTPException(status_code=500, detail="Battery monitoring service not initialized")
    
    try:
        metrics = await battery_service.get_fleet_battery_metrics(fleet_id)
        
        return FleetBatteryResponse(
            fleet_id=fleet_id,
            drones=metrics
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting fleet battery metrics: {str(e)}")


@router.post("/battery/thresholds/{drone_id}", response_model=BatteryThresholdsResponse)
async def set_battery_thresholds(
    drone_id: str,
    thresholds: BatteryThresholdsRequest,
    current_user: str = Depends(get_current_user)
):
    """Set battery alert thresholds for a specific drone."""
    if not battery_service:
        raise HTTPException(status_code=500, detail="Battery monitoring service not initialized")
    
    try:
        result = await battery_service.set_battery_thresholds(
            drone_id=drone_id,
            warning_threshold=thresholds.warning_threshold,
            critical_threshold=thresholds.critical_threshold,
            temperature_max=thresholds.temperature_max,
            voltage_min=thresholds.voltage_min,
            auto_return_threshold=thresholds.auto_return_threshold
        )
        
        return BatteryThresholdsResponse(**result.dict())
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error setting battery thresholds: {str(e)}")


@router.get("/battery/thresholds/{drone_id}", response_model=BatteryThresholdsResponse)
async def get_battery_thresholds(
    drone_id: str,
    current_user: str = Depends(get_current_user)
):
    """Get battery alert thresholds for a specific drone."""
    if not battery_service:
        raise HTTPException(status_code=500, detail="Battery monitoring service not initialized")
    
    try:
        thresholds = await battery_service.get_battery_thresholds(drone_id)
        
        return BatteryThresholdsResponse(**thresholds.dict())
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting battery thresholds: {str(e)}")


@router.post("/energy/predict", response_model=EnergyPredictionResponse)
async def predict_energy_consumption(
    request: EnergyPredictionRequest,
    current_user: str = Depends(get_current_user)
):
    """Predict energy consumption for a mission."""
    if not energy_prediction_service:
        raise HTTPException(status_code=500, detail="Energy prediction service not initialized")
    
    try:
        prediction = await energy_prediction_service.predict_energy_consumption(
            mission_id=request.mission_id,
            drone_id=request.drone_id,
            waypoints=request.waypoints,
            payload_weight=request.payload_weight,
            wind_speed=request.wind_speed,
            wind_direction=request.wind_direction,
            temperature=request.temperature,
            model_type=request.model_type or EnergyPredictionModel.LINEAR
        )
        
        return EnergyPredictionResponse(**prediction.dict())
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error predicting energy consumption: {str(e)}")


@router.post("/mission/optimize", response_model=OptimizeMissionResponse)
async def optimize_mission(
    request: OptimizeMissionRequest,
    current_user: str = Depends(get_current_user)
):
    """Optimize a mission for energy efficiency."""
    if not energy_prediction_service:
        raise HTTPException(status_code=500, detail="Energy prediction service not initialized")
    
    try:
        prediction, optimized_waypoints = await energy_prediction_service.optimize_mission(
            mission_id=request.mission_id,
            drone_id=request.drone_id,
            current_battery=request.current_battery,
            optimization_level=request.optimization_level or 0.5
        )
        
        # This is a placeholder - in a real implementation, you would calculate these values
        original_consumption = 80.0
        original_duration = 600
        
        return OptimizeMissionResponse(
            mission_id=request.mission_id,
            original_consumption=original_consumption,
            optimized_consumption=prediction.estimated_consumption,
            savings=original_consumption - prediction.estimated_consumption,
            original_duration=original_duration,
            optimized_duration=prediction.estimated_duration,
            modifications=prediction.recommendations or [],
            optimized_waypoints=optimized_waypoints
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except NotImplementedError as e:
        raise HTTPException(status_code=501, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error optimizing mission: {str(e)}")


@router.post("/battery/health", response_model=BatteryHealthResponse)
async def get_battery_health(
    request: BatteryHealthRequest,
    current_user: str = Depends(get_current_user)
):
    """Get battery health analysis."""
    if not battery_service:
        raise HTTPException(status_code=500, detail="Battery monitoring service not initialized")
    
    try:
        health_record = await battery_service.get_battery_health(
            drone_id=request.drone_id,
            battery_id=request.battery_id
        )
        
        if not health_record:
            raise HTTPException(status_code=404, detail=f"Battery health data for drone {request.drone_id} not found")
        
        # This is a placeholder - in a real implementation, you would get historical data
        history = [health_record]
        
        return BatteryHealthResponse(
            drone_id=request.drone_id,
            battery_id=health_record.battery_id,
            current_health=health_record.health_percent,
            degradation_rate=0.05,  # 0.05% per cycle
            estimated_remaining_cycles=health_record.estimated_remaining_cycles or 450,
            recommendation=health_record.recommendation or "Battery is in good condition",
            history=history
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting battery health: {str(e)}")


@router.post("/battery/trigger-rth/{drone_id}")
async def trigger_return_to_home(
    drone_id: str,
    reason: str = "Manual RTH trigger",
    current_user: str = Depends(get_current_user)
):
    """Manually trigger return-to-home for a drone."""
    if not battery_service:
        raise HTTPException(status_code=500, detail="Battery monitoring service not initialized")
    
    try:
        success = await battery_service.trigger_return_to_home(drone_id, reason)
        
        if not success:
            raise HTTPException(status_code=400, detail=f"Failed to trigger RTH for drone {drone_id}")
        
        return {"message": f"Return-to-home triggered for drone {drone_id}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error triggering RTH: {str(e)}")


@router.post("/battery/start-monitoring")
async def start_battery_monitoring(
    background_tasks: BackgroundTasks,
    current_user: str = Depends(get_current_user)
):
    """Start the battery monitoring service."""
    if not battery_service:
        raise HTTPException(status_code=500, detail="Battery monitoring service not initialized")
    
    background_tasks.add_task(battery_service.start)
    
    return {"message": "Battery monitoring service starting"}
