"""
API router for the TACS module.
"""

from fastapi import APIRouter

from api.endpoints import (
    targets,
    tracks,
    sensors,
    fusion,
    coordination,
    analytics
)

api_router = APIRouter()

# Include all endpoint routers
api_router.include_router(targets.router, prefix="/targets", tags=["targets"])
api_router.include_router(tracks.router, prefix="/tracks", tags=["tracks"])
api_router.include_router(sensors.router, prefix="/sensors", tags=["sensors"])
api_router.include_router(fusion.router, prefix="/fusion", tags=["fusion"])
api_router.include_router(coordination.router, prefix="/coordination", tags=["coordination"])
api_router.include_router(analytics.router, prefix="/analytics", tags=["analytics"])
