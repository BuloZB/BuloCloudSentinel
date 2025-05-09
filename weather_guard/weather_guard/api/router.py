"""
API router for Weather Guard service.

This module provides the main API router for the Weather Guard service,
which includes all API endpoints.
"""

from fastapi import APIRouter

from weather_guard.api.endpoints import weather, alerts
from weather_guard.core.config import settings

api_router = APIRouter()

# Include API endpoints
api_router.include_router(weather.router, prefix="/weather", tags=["weather"])
api_router.include_router(alerts.router, prefix="/alerts", tags=["alerts"])
