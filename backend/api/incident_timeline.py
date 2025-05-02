from fastapi import APIRouter, Query, HTTPException

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
from typing import List, Optional
from pydantic import BaseModel
from datetime import datetime

router = APIRouter(
    prefix="/incident-timeline",
    tags=["Incident Timeline"]
)

class IncidentEvent(BaseModel):
    id: int
    timestamp: datetime
    label: str
    thumbnail_url: Optional[str]
    metadata: dict

# Dummy in-memory store for demonstration
incident_events_db = [
    IncidentEvent(
        id=1,
        timestamp=datetime(2025, 4, 20, 12, 0, 0),
        label="Person detected",
        thumbnail_url="https://example.com/thumb1.jpg",
        metadata={"confidence": 0.95, "type": "person"}
    ),
    IncidentEvent(
        id=2,
        timestamp=datetime(2025, 4, 20, 12, 5, 0),
        label="Vehicle detected",
        thumbnail_url="https://example.com/thumb2.jpg",
        metadata={"confidence": 0.89, "type": "vehicle"}
    ),
]

@router.get("/", response_model=List[IncidentEvent])
async def get_incident_events(
    start_time: Optional[datetime] = Query(None, description="Start time for filtering events"),
    end_time: Optional[datetime] = Query(None, description="End time for filtering events"),
    label_filter: Optional[str] = Query(None, description="Filter events by label"),
    search: Optional[str] = Query(None, description="Full-text search on metadata")
):
    results = incident_events_db
    if start_time:
        results = [e for e in results if e.timestamp >= start_time]
    if end_time:
        results = [e for e in results if e.timestamp <= end_time]
    if label_filter:
        results = [e for e in results if label_filter.lower() in e.label.lower()]
    if search:
        results = [e for e in results if search.lower() in str(e.metadata).lower()]
    return results
