from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Any, Dict
from backend.sensor_fusion.engine import SensorFusionEngine
import asyncio

router = APIRouter()
fusion_engine = SensorFusionEngine()

class SensorDataIngestRequest(BaseModel):
    sensor_type: str
    data: Dict[str, Any]

@router.post("/sensor-fusion/ingest")
async def ingest_sensor_data(request: SensorDataIngestRequest):
    fusion_engine.ingest_data(request.sensor_type, request.data)
    return {"detail": "Data ingested"}

@router.get("/sensor-fusion/fused-data")
async def get_fused_data():
    fused = await fusion_engine.fuse_data()
    if fused is None:
        raise HTTPException(status_code=404, detail="No fused data available")
    return fused

# Background task to run fusion engine continuously
async def fusion_engine_runner():
    await fusion_engine.run()
