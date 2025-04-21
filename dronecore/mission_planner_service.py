from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import List, Optional
import uuid

router = APIRouter()

class Waypoint(BaseModel):
    latitude: float
    longitude: float
    altitude: Optional[float] = None
    action: Optional[str] = None
    order: int

class MissionPlan(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: Optional[str] = None
    waypoints: List[Waypoint]

# In-memory storage for demo purposes
missions = {}

@router.post("/missions", response_model=MissionPlan)
async def create_mission(mission: MissionPlan):
    if mission.id in missions:
        raise HTTPException(status_code=400, detail="Mission with this ID already exists")
    missions[mission.id] = mission
    return mission

@router.get("/missions", response_model=List[MissionPlan])
async def list_missions():
    return list(missions.values())

@router.get("/missions/{mission_id}", response_model=MissionPlan)
async def get_mission(mission_id: str):
    mission = missions.get(mission_id)
    if not mission:
        raise HTTPException(status_code=404, detail="Mission not found")
    return mission

@router.put("/missions/{mission_id}", response_model=MissionPlan)
async def update_mission(mission_id: str, mission_update: MissionPlan):
    if mission_id != mission_update.id:
        raise HTTPException(status_code=400, detail="Mission ID mismatch")
    if mission_id not in missions:
        raise HTTPException(status_code=404, detail="Mission not found")
    missions[mission_id] = mission_update
    return mission_update

@router.delete("/missions/{mission_id}")
async def delete_mission(mission_id: str):
    if mission_id not in missions:
        raise HTTPException(status_code=404, detail="Mission not found")
    del missions[mission_id]
    return {"detail": "Mission deleted"}

# Simulation endpoint (mocked telemetry)
@router.get("/missions/{mission_id}/simulate")
async def simulate_mission(mission_id: str):
    mission = missions.get(mission_id)
    if not mission:
        raise HTTPException(status_code=404, detail="Mission not found")
    # Mock telemetry data for simulation
    telemetry = []
    for wp in mission.waypoints:
        telemetry.append({
            "latitude": wp.latitude,
            "longitude": wp.longitude,
            "altitude": wp.altitude or 0,
            "action": wp.action or "none",
            "status": "reached"
        })
    return {"mission_id": mission_id, "telemetry": telemetry}
