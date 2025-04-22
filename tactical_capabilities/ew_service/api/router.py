"""
API router for the EW service.
"""

from fastapi import APIRouter

from api.endpoints import (
    platforms,
    attacks,
    protections,
    supports,
    spectrum,
    threats,
    countermeasures,
    waveforms,
)

api_router = APIRouter()

# Include all endpoint routers
api_router.include_router(platforms.router, prefix="/platforms", tags=["platforms"])
api_router.include_router(attacks.router, prefix="/attacks", tags=["attacks"])
api_router.include_router(protections.router, prefix="/protections", tags=["protections"])
api_router.include_router(supports.router, prefix="/support", tags=["support"])
api_router.include_router(spectrum.router, prefix="/spectrum", tags=["spectrum"])
api_router.include_router(threats.router, prefix="/threats", tags=["threats"])
api_router.include_router(countermeasures.router, prefix="/countermeasures", tags=["countermeasures"])
api_router.include_router(waveforms.router, prefix="/waveforms", tags=["waveforms"])
