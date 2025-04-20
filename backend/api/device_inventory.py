from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
from backend.api.dependencies import get_current_user

router = APIRouter(
    prefix="/device-inventory",
    tags=["Device Inventory & Health Status"]
)

class Device(BaseModel):
    device_id: str
    device_type: str  # e.g. camera, sensor, drone
    zone: Optional[str]
    ip_address: Optional[str]
    online: bool
    last_activity: Optional[datetime]
    critical: bool

# In-memory store for demo purposes
devices_db = {}

@router.get("/devices", response_model=List[Device])
async def list_devices(current_user: str = Depends(get_current_user)):
    return list(devices_db.values())

@router.post("/device/register", response_model=Device)
async def register_device(device: Device, current_user: str = Depends(get_current_user)):
    devices_db[device.device_id] = device
    return device

@router.patch("/device/config/{device_id}", response_model=Device)
async def update_device_config(device_id: str, device_update: Device, current_user: str = Depends(get_current_user)):
    if device_id not in devices_db:
        raise HTTPException(status_code=404, detail="Device not found")
    stored_device = devices_db[device_id]
    update_data = device_update.dict(exclude_unset=True)
    updated_device = stored_device.copy(update=update_data)
    devices_db[device_id] = updated_device
    return updated_device
