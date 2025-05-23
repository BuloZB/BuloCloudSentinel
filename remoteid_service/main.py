"""
Remote ID & Regulatory Compliance Service for Bulo.Cloud Sentinel

This module serves as the entry point for the Remote ID service, which provides
Remote ID broadcasting, flight plan submission, and NOTAM integration capabilities.
"""

import asyncio
import logging
import os
from contextlib import asynccontextmanager
from typing import Dict, List, Optional

import uvicorn
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.staticfiles import StaticFiles

# Import local modules
from remoteid_service.core.settings import get_settings, Settings
from remoteid_service.core.security import setup_security
from remoteid_service.api.router import api_router
from remoteid_service.db.session import create_db_and_tables, get_db_session
from remoteid_service.services.remoteid_broadcast import RemoteIDBroadcastService
from remoteid_service.services.remoteid_logging import RemoteIDLoggingService
from remoteid_service.services.flightplan_service import FlightPlanService
from remoteid_service.services.notam_service import NOTAMService

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("remoteid_service")

# Application lifespan context manager
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for the FastAPI application.
    
    This function handles startup and shutdown events for the application.
    """
    # Startup: Initialize services and database
    logger.info("Starting Remote ID & Regulatory Compliance Service")
    
    # Create database tables
    await create_db_and_tables()
    
    # Initialize services
    settings = get_settings()
    
    # Start Remote ID broadcasting service if enabled
    if settings.ENABLE_BROADCAST:
        app.state.broadcast_service = RemoteIDBroadcastService(settings)
        await app.state.broadcast_service.start()
        logger.info("Remote ID broadcasting service started")
    
    # Initialize Remote ID logging service
    app.state.logging_service = RemoteIDLoggingService(settings)
    await app.state.logging_service.start()
    logger.info("Remote ID logging service started")
    
    # Initialize flight plan service
    app.state.flightplan_service = FlightPlanService(settings)
    await app.state.flightplan_service.start()
    logger.info("Flight plan service started")
    
    # Initialize NOTAM service
    app.state.notam_service = NOTAMService(settings)
    await app.state.notam_service.start()
    logger.info("NOTAM service started")
    
    logger.info("All services started successfully")
    
    yield
    
    # Shutdown: Clean up resources
    logger.info("Shutting down Remote ID & Regulatory Compliance Service")
    
    # Stop services
    if hasattr(app.state, "broadcast_service"):
        await app.state.broadcast_service.stop()
        logger.info("Remote ID broadcasting service stopped")
    
    await app.state.logging_service.stop()
    logger.info("Remote ID logging service stopped")
    
    await app.state.flightplan_service.stop()
    logger.info("Flight plan service stopped")
    
    await app.state.notam_service.stop()
    logger.info("NOTAM service stopped")
    
    logger.info("All services stopped successfully")

# Create FastAPI application
app = FastAPI(
    title="Remote ID & Regulatory Compliance Service",
    description="API for Remote ID broadcasting, flight plan submission, and NOTAM integration",
    version="1.0.0",
    lifespan=lifespan,
)

# Set up security
setup_security(app)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API router
app.include_router(api_router, prefix="/api/v1")

# Health check endpoint
@app.get("/health", tags=["Health"])
async def health_check():
    """
    Health check endpoint.
    
    Returns:
        dict: Health status
    """
    return {"status": "healthy", "service": "remoteid"}

# Root endpoint
@app.get("/", tags=["Root"])
async def root():
    """
    Root endpoint.
    
    Returns:
        dict: Service information
    """
    return {
        "service": "Remote ID & Regulatory Compliance Service",
        "version": "1.0.0",
        "documentation": "/docs",
    }

# Error handlers
@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    """
    Handle HTTP exceptions.
    
    Args:
        request: Request object
        exc: Exception object
        
    Returns:
        JSONResponse: Error response
    """
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail},
    )

@app.exception_handler(Exception)
async def general_exception_handler(request, exc):
    """
    Handle general exceptions.
    
    Args:
        request: Request object
        exc: Exception object
        
    Returns:
        JSONResponse: Error response
    """
    logger.error(f"Unhandled exception: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={"detail": "Internal server error"},
    )

# Run the application
if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8080,
        reload=True,
    )
