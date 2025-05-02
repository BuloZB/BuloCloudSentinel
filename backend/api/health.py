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

@router.get("/health")
async def health_check():
    # Basic health check endpoint
    return {"status": "ok"}
