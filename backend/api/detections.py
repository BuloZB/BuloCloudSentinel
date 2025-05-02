from fastapi import APIRouter

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

router = APIRouter()

@router.get("/")
async def get_detections():
    # Placeholder for detections retrieval
    return {"detections": []}
