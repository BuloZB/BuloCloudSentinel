from fastapi import APIRouter, Depends, HTTPException, status
from typing import List, Optional
from pydantic import BaseModel
from backend.api.dependencies import get_user_repository
from backend.domain.repositories.user_repository import IUserRepository

router = APIRouter()

class Waypoint(BaseModel):
    latitude: float
    longitude: float
    altitude: float

class Mission(BaseModel):
    id: int
    name: str
    waypoints: List[Waypoint]

# In-memory mission store for demonstration
missions = {}

@router.post("/missions", status_code=status.HTTP_201_CREATED)
async def create_mission(mission: Mission, user_repo: IUserRepository = Depends(get_user_repository)):
    if mission.id in missions:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Mission ID already exists")
    missions[mission.id] = mission
    return {"message": "Mission created"}

@router.get("/missions/{mission_id}", response_model=Optional[Mission])
async def get_mission(mission_id: int, user_repo: IUserRepository = Depends(get_user_repository)):
    return missions.get(mission_id)

@router.delete("/missions/{mission_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_mission(mission_id: int, user_repo: IUserRepository = Depends(get_user_repository)):
    if mission_id in missions:
        del missions[mission_id]
    else:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Mission not found")
