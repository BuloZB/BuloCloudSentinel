import time
from prometheus_client import Counter, Histogram, generate_latest, CONTENT_TYPE_LATEST
from fastapi import APIRouter, Request, Response
from fastapi.responses import StreamingResponse

router = APIRouter(
    prefix="/monitoring",
    tags=["Monitoring"]
)

REQUEST_COUNT = Counter('bulo_sentinel_ai_request_count', 'Total number of requests')
REQUEST_LATENCY = Histogram('bulo_sentinel_ai_request_latency_seconds', 'Request latency in seconds')

@router.middleware("http")
async def metrics_middleware(request: Request, call_next):
    REQUEST_COUNT.inc()
    start_time = time.time()
    response = await call_next(request)
    latency = time.time() - start_time
    REQUEST_LATENCY.observe(latency)
    return response

@router.get("/metrics")
async def metrics():
    data = generate_latest()
    return Response(content=data, media_type=CONTENT_TYPE_LATEST)
