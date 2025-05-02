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

router = APIRouter(
    prefix="/me",
    tags=["User Info"]
)

@router.get("/")
async def get_current_user_info(current_user: str = Depends(get_current_user)):
    # Placeholder for returning user info
    return {"username": current_user, "roles": ["user"]}
