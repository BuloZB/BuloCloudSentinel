from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import uuid

router = APIRouter()

class AutonomousMission(BaseModel):
    id: str = None
    name: str
    description: Optional[str] = None
    parameters: dict
    status: str = "pending"

# In-memory storage for demo purposes
missions = {}

@router.post("/autonomous-missions", response_model=AutonomousMission)
async def create_mission(mission: AutonomousMission):
    if not mission.id:
        mission.id = str(uuid.uuid4())
    if mission.id in missions:
        raise HTTPException(status_code=400, detail="Mission with this ID already exists")
    missions[mission.id] = mission
    return mission

@router.get("/autonomous-missions", response_model=List[AutonomousMission])
async def list_missions():
    return list(missions.values())

@router.get("/autonomous-missions/{mission_id}", response_model=AutonomousMission)
async def get_mission(mission_id: str):
    mission = missions.get(mission_id)
    if not mission:
        raise HTTPException(status_code=404, detail="Mission not found")
    return mission

@router.put("/autonomous-missions/{mission_id}", response_model=AutonomousMission)
async def update_mission(mission_id: str, mission_update: AutonomousMission):
    if mission_id != mission_update.id:
        raise HTTPException(status_code=400, detail="Mission ID mismatch")
    if mission_id not in missions:
        raise HTTPException(status_code=404, detail="Mission not found")
    missions[mission_id] = mission_update
    return mission_update

@router.delete("/autonomous-missions/{mission_id}")
async def delete_mission(mission_id: str):
    if mission_id not in missions:
        raise HTTPException(status_code=404, detail="Mission not found")
    del missions[mission_id]
    return {"detail": "Mission deleted"}

@router.post("/autonomous-missions/{mission_id}/simulate")
async def simulate_mission(mission_id: str):
    mission = missions.get(mission_id)
    if not mission:
        raise HTTPException(status_code=404, detail="Mission not found")
    # Placeholder for simulation logic
    return {"mission_id": mission_id, "simulation": "Simulation data placeholder"}
