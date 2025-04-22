"""
API router for the SIGINT service.
"""

from fastapi import APIRouter

from api.endpoints import (
    collectors,
    detections,
    sources,
    recordings,
    analysis,
    alerts,
    intelligence,
    profiles,
)

api_router = APIRouter()

# Include all endpoint routers
api_router.include_router(collectors.router, prefix="/collectors", tags=["collectors"])
api_router.include_router(detections.router, prefix="/detections", tags=["detections"])
api_router.include_router(sources.router, prefix="/sources", tags=["sources"])
api_router.include_router(recordings.router, prefix="/recordings", tags=["recordings"])
api_router.include_router(analysis.router, prefix="/analysis", tags=["analysis"])
api_router.include_router(alerts.router, prefix="/alerts", tags=["alerts"])
api_router.include_router(intelligence.router, prefix="/intelligence", tags=["intelligence"])
api_router.include_router(profiles.router, prefix="/profiles", tags=["profiles"])
