"""
Health check endpoints for the Model Hub service.

This module provides API endpoints for checking the health of the Model Hub service.
"""

import logging
import os
from typing import Dict, Any

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import JSONResponse

# Setup logging
logger = logging.getLogger(__name__)

# Create router
router = APIRouter()

@router.get("/")
async def health_check():
    """
    Health check endpoint.
    
    Returns:
        Health status
    """
    return {
        "status": "ok",
        "service": "model-hub",
        "version": "0.1.0",
    }

@router.get("/readiness")
async def readiness_check():
    """
    Readiness check endpoint.
    
    This endpoint checks if the service is ready to handle requests.
    
    Returns:
        Readiness status
    """
    # Check if MLflow is available
    mlflow_uri = os.environ.get("MLFLOW_TRACKING_URI")
    if not mlflow_uri:
        return JSONResponse(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            content={
                "status": "error",
                "message": "MLflow tracking URI not configured",
            },
        )
    
    # Check if database is available
    db_url = os.environ.get("DATABASE_URL")
    if not db_url:
        return JSONResponse(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            content={
                "status": "error",
                "message": "Database URL not configured",
            },
        )
    
    return {
        "status": "ok",
        "service": "model-hub",
        "version": "0.1.0",
        "dependencies": {
            "mlflow": {
                "status": "ok",
                "uri": mlflow_uri,
            },
            "database": {
                "status": "ok",
                "uri": db_url.split("@")[1] if "@" in db_url else db_url,
            },
        },
    }

@router.get("/liveness")
async def liveness_check():
    """
    Liveness check endpoint.
    
    This endpoint checks if the service is alive.
    
    Returns:
        Liveness status
    """
    return {
        "status": "ok",
        "service": "model-hub",
        "version": "0.1.0",
    }
