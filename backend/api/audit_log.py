from fastapi import APIRouter, Request, Depends, HTTPException, status
from pydantic import BaseModel
from datetime import datetime
from backend.api.dependencies import verify_jwt_token
from typing import List

router = APIRouter()

class AuditLogEntry(BaseModel):
    timestamp: datetime
    username: str
    action: str
    resource: str
    details: str

# In-memory audit log storage for demo purposes
audit_logs: List[AuditLogEntry] = []

@router.post("/audit-log")
async def log_event(entry: AuditLogEntry, token=Depends(verify_jwt_token)):
    audit_logs.append(entry)
    return {"detail": "Event logged"}

@router.get("/audit-log", response_model=List[AuditLogEntry])
async def get_audit_logs(token=Depends(verify_jwt_token)):
    # For simplicity, return all logs; in production, add filtering and pagination
    return audit_logs
