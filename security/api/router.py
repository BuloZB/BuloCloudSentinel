"""
Security API router for Bulo.Cloud Sentinel.

This module provides API endpoints for security-related functionality.
"""

from fastapi import APIRouter

from security.api import csp_report

# Create security API router
router = APIRouter()

# Include security API endpoints
router.include_router(csp_report.router, prefix="/security", tags=["Security"])