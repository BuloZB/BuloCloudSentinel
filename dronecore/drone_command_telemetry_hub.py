from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from typing import Optional
from fastapi.security import OAuth2PasswordBearer
from datetime import datetime

router = APIRouter(
    prefix="/drone-hub",
    tags=["Drone Command & Telemetry Hub"]
)

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

class TelemetryData(BaseModel):
    drone_id: str
    latitude: float
    longitude: float
    altitude: float
    speed: float
    battery: float
    timestamp: datetime

class Command(BaseModel):
    drone_id: str
    command: str  # e.g. "takeoff", "waypoint", "return_home"
    parameters: Optional[dict] = None

# In-memory stores for demo purposes
telemetry_store = {}
command_store = []

def verify_token(token: str = Depends(oauth2_scheme)):
    # Placeholder for JWT validation
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return token

@router.post("/telemetry")
async def receive_telemetry(data: TelemetryData, token: str = Depends(verify_token)):
    telemetry_store[data.drone_id] = data
    return {"status": "telemetry received"}

@router.get("/telemetry/{drone_id}", response_model=Optional[TelemetryData])
async def get_telemetry(drone_id: str, token: str = Depends(verify_token)):
    return telemetry_store.get(drone_id)

@router.post("/command")
async def send_command(cmd: Command, token: str = Depends(verify_token)):
    command_store.append(cmd)
    return {"status": "command sent"}
