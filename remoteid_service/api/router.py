"""
API router for the Remote ID & Regulatory Compliance Service.

This module defines the API router for the service, which includes all endpoints.
"""

from fastapi import APIRouter

from remoteid_service.api.endpoints import (
    auth,
    remoteid,
    flightplans,
    notams,
)

# Create API router
api_router = APIRouter()

# Include endpoint routers
api_router.include_router(auth.router, prefix="/auth", tags=["Authentication"])
api_router.include_router(remoteid.router, prefix="/remoteid", tags=["Remote ID"])
api_router.include_router(flightplans.router, prefix="/flightplans", tags=["Flight Plans"])
api_router.include_router(notams.router, prefix="/notams", tags=["NOTAMs"])
