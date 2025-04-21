from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from pydantic import BaseModel, Field
from typing import Any, Dict, List, Optional
from backend.sensor_fusion.engine import SensorFusionEngine, SensorType
from backend.api.dependencies import get_current_user
import asyncio

router = APIRouter(prefix="/sensor-fusion", tags=["Sensor Fusion"])
fusion_engine = SensorFusionEngine()

class SensorRegistrationRequest(BaseModel):
    sensor_id: str = Field(..., description="Unique identifier for the sensor")
    sensor_type: str = Field(..., description="Type of sensor (camera, radar, lidar, etc.)")
    capabilities: List[str] = Field(default=[], description="List of sensor capabilities")

class SensorDataIngestRequest(BaseModel):
    sensor_id: str = Field(..., description="ID of the registered sensor")
    data: Dict[str, Any] = Field(..., description="Sensor data payload")

class FusionFilterParams(BaseModel):
    include_position: bool = Field(True, description="Include position data in response")
    include_detections: bool = Field(True, description="Include detection data in response")
    include_classifications: bool = Field(True, description="Include classification data in response")
    include_telemetry: bool = Field(True, description="Include telemetry data in response")
    include_sensors: bool = Field(True, description="Include sensor metadata in response")

@router.post("/register", status_code=201)
async def register_sensor(request: SensorRegistrationRequest, current_user: str = Depends(get_current_user)):
    """Register a new sensor with the fusion engine."""
    try:
        result = fusion_engine.register_sensor(
            sensor_id=request.sensor_id,
            sensor_type=request.sensor_type,
            capabilities=request.capabilities
        )
        return result
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.post("/ingest")
async def ingest_sensor_data(request: SensorDataIngestRequest, current_user: str = Depends(get_current_user)):
    """Ingest data from a registered sensor."""
    try:
        result = fusion_engine.ingest_data(request.sensor_id, request.data)
        return result
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.get("/fused-data")
async def get_fused_data(
    include_position: bool = True,
    include_detections: bool = True,
    include_classifications: bool = True,
    include_telemetry: bool = True,
    include_sensors: bool = True,
    current_user: str = Depends(get_current_user)
):
    """Get the latest fused data from all sensors."""
    fused = await fusion_engine.fuse_data()
    if not fused:
        raise HTTPException(status_code=404, detail="No fused data available")

    # Filter response based on query parameters
    result = {}
    if "timestamp" in fused:
        result["timestamp"] = fused["timestamp"]

    if include_position and "position" in fused:
        result["position"] = fused["position"]

    if include_detections and "detections" in fused:
        result["detections"] = fused["detections"]

    if include_classifications and "classifications" in fused:
        result["classifications"] = fused["classifications"]

    if include_telemetry and "telemetry" in fused:
        result["telemetry"] = fused["telemetry"]

    if include_sensors and "sensors" in fused:
        result["sensors"] = fused["sensors"]

    return result

@router.get("/sensors")
async def get_sensors(current_user: str = Depends(get_current_user)):
    """Get information about all registered sensors."""
    fused = await fusion_engine.fuse_data()
    if not fused or "sensors" not in fused:
        return {"sensors": {}}

    return {"sensors": fused["sensors"]}

@router.get("/sensors/{sensor_id}")
async def get_sensor(sensor_id: str, current_user: str = Depends(get_current_user)):
    """Get information about a specific sensor."""
    fused = await fusion_engine.fuse_data()
    if not fused or "sensors" not in fused or sensor_id not in fused["sensors"]:
        raise HTTPException(status_code=404, detail=f"Sensor {sensor_id} not found")

    return {"sensor": fused["sensors"][sensor_id]}

@router.get("/position")
async def get_position(current_user: str = Depends(get_current_user)):
    """Get the latest fused position data."""
    fused = await fusion_engine.fuse_data()
    if not fused or "position" not in fused or not fused["position"]:
        raise HTTPException(status_code=404, detail="No position data available")

    return {"position": fused["position"]}

@router.get("/detections")
async def get_detections(current_user: str = Depends(get_current_user)):
    """Get the latest fused detection data."""
    fused = await fusion_engine.fuse_data()
    if not fused or "detections" not in fused or not fused["detections"]:
        raise HTTPException(status_code=404, detail="No detection data available")

    return {"detections": fused["detections"]}

# Background task to run fusion engine continuously
async def fusion_engine_runner():
    """Background task to run the fusion engine continuously."""
    await fusion_engine.run()

# Start the fusion engine in the background when the API starts
@router.on_event("startup")
async def startup_event():
    """Start the fusion engine when the API starts."""
    # Start the fusion engine in a background task
    asyncio.create_task(fusion_engine_runner())
