"""
API router for the ISR service.
"""

from fastapi import APIRouter

from api.endpoints import (
    sensors,
    platforms,
    observations,
    detections,
    targets,
    surveillance_areas,
    alerts,
    fusion,
)

api_router = APIRouter()

# Include all endpoint routers
api_router.include_router(sensors.router, prefix="/sensors", tags=["sensors"])
api_router.include_router(platforms.router, prefix="/platforms", tags=["platforms"])
api_router.include_router(observations.router, prefix="/observations", tags=["observations"])
api_router.include_router(detections.router, prefix="/detections", tags=["detections"])
api_router.include_router(targets.router, prefix="/targets", tags=["targets"])
api_router.include_router(surveillance_areas.router, prefix="/surveillance-areas", tags=["surveillance-areas"])
api_router.include_router(alerts.router, prefix="/alerts", tags=["alerts"])
api_router.include_router(fusion.router, prefix="/fusion", tags=["fusion"])
