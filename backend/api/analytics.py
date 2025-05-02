from fastapi import APIRouter, Depends

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
from backend.api.dependencies import get_current_user
from typing import Dict, Any
import random

router = APIRouter(prefix="/analytics", tags=["Analytics"])

@router.get("/overview")
async def get_overview(current_user=Depends(get_current_user)) -> Dict[str, Any]:
    # Placeholder: return random analytics data
    return {
        "active_missions": random.randint(1, 10),
        "alerts_last_24h": random.randint(0, 50),
        "device_health": {
            "cameras_online": random.randint(10, 20),
            "drones_online": random.randint(1, 5),
            "sensors_online": random.randint(5, 15)
        },
        "average_response_time_ms": random.uniform(100, 500)
    }
