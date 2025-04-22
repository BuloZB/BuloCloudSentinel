"""
API router for the SentinelBeacon module.
"""

from fastapi import APIRouter

from api.endpoints import (
    nodes,
    messages,
    channels,
    network,
    beacon,
    telemetry,
)

api_router = APIRouter()

# Include all endpoint routers
api_router.include_router(nodes.router, prefix="/nodes", tags=["nodes"])
api_router.include_router(messages.router, prefix="/messages", tags=["messages"])
api_router.include_router(channels.router, prefix="/channels", tags=["channels"])
api_router.include_router(network.router, prefix="/network", tags=["network"])
api_router.include_router(beacon.router, prefix="/beacon", tags=["beacon"])
api_router.include_router(telemetry.router, prefix="/telemetry", tags=["telemetry"])
