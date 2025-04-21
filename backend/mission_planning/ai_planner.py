from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import uuid

router = APIRouter()

class AIPlanRequest(BaseModel):
    mission_id: str
    sensor_data: dict

class AIPlanResponse(BaseModel):
    mission_id: str
    updated_parameters: dict
    status: str

# In-memory storage for demo purposes
ai_plans = {}

@router.post("/ai-planner/plan", response_model=AIPlanResponse)
async def plan_mission(request: AIPlanRequest):
    # Placeholder for AI decision-making logic
    if request.mission_id not in ai_plans:
        ai_plans[request.mission_id] = {
            "parameters": {},
            "status": "planned"
        }
    # Simulate updating mission parameters based on sensor data
    updated_params = {"adaptive_speed": "medium", "route_adjustment": "none"}
    ai_plans[request.mission_id]["parameters"] = updated_params
    ai_plans[request.mission_id]["status"] = "updated"
    return AIPlanResponse(
        mission_id=request.mission_id,
        updated_parameters=updated_params,
        status=ai_plans[request.mission_id]["status"]
    )

@router.get("/ai-planner/status/{mission_id}", response_model=AIPlanResponse)
async def get_plan_status(mission_id: str):
    plan = ai_plans.get(mission_id)
    if not plan:
        raise HTTPException(status_code=404, detail="Plan not found")
    return AIPlanResponse(
        mission_id=mission_id,
        updated_parameters=plan["parameters"],
        status=plan["status"]
    )
