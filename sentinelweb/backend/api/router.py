"""
SentinelWeb Backend - API Router

This module sets up the main API router and includes all endpoint routers.
"""

from fastapi import APIRouter

from backend.api.endpoints import (
    auth,
    users,
    drones,
    missions,
    telemetry,
    dashboard,
    plugins,
    widgets,
    settings
)

# Create main API router
api_router = APIRouter()

# Include all endpoint routers
api_router.include_router(auth.router, prefix="/auth", tags=["Authentication"])
api_router.include_router(users.router, prefix="/users", tags=["Users"])
api_router.include_router(drones.router, prefix="/drones", tags=["Drones"])
api_router.include_router(missions.router, prefix="/missions", tags=["Missions"])
api_router.include_router(telemetry.router, prefix="/telemetry", tags=["Telemetry"])
api_router.include_router(dashboard.router, prefix="/dashboard", tags=["Dashboard"])
api_router.include_router(plugins.router, prefix="/plugins", tags=["Plugins"])
api_router.include_router(widgets.router, prefix="/widgets", tags=["Widgets"])
api_router.include_router(settings.router, prefix="/settings", tags=["Settings"])
