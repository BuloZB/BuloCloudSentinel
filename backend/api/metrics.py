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
from prometheus_client import Counter, generate_latest, CONTENT_TYPE_LATEST
from fastapi.responses import Response

router = APIRouter(
    prefix="/metrics",
    tags=["Metrics"]
)

# Define Prometheus metrics
REQUEST_COUNT = Counter('request_count', 'Total number of requests')

@router.get("/")
async def metrics():
    REQUEST_COUNT.inc()
    data = generate_latest()
    return Response(content=data, media_type=CONTENT_TYPE_LATEST)
