"""
Electronic Warfare (EW) Service

This module provides the main FastAPI application for the EW service.
"""

import logging
import os
from contextlib import asynccontextmanager
from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware

# Import local modules
from api.router import api_router
from core.config import settings
from core.security import get_current_user
from db.session import create_db_and_tables
from services.platform_manager import PlatformManager
from services.attack_manager import AttackManager
from services.protection_manager import ProtectionManager
from services.support_manager import SupportManager
from services.spectrum_manager import SpectrumManager
from services.threat_manager import ThreatManager
from services.countermeasure_manager import CountermeasureManager
from services.waveform_manager import WaveformManager
from services.event_publisher import EventPublisher
from services.storage_manager import StorageManager

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Lifespan context manager for startup/shutdown events
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize services and connections
    logger.info("Starting EW service")
    
    # Create database tables if they don't exist
    await create_db_and_tables()
    
    # Initialize services
    app.state.platform_manager = PlatformManager()
    app.state.attack_manager = AttackManager()
    app.state.protection_manager = ProtectionManager()
    app.state.support_manager = SupportManager()
    app.state.spectrum_manager = SpectrumManager()
    app.state.threat_manager = ThreatManager()
    app.state.countermeasure_manager = CountermeasureManager()
    app.state.waveform_manager = WaveformManager()
    app.state.storage_manager = StorageManager(
        endpoint=settings.MINIO_ENDPOINT,
        access_key=settings.MINIO_ACCESS_KEY,
        secret_key=settings.MINIO_SECRET_KEY,
        bucket_name=settings.MINIO_BUCKET_NAME,
        secure=settings.MINIO_SECURE
    )
    app.state.event_publisher = EventPublisher(
        host=settings.RABBITMQ_HOST,
        port=settings.RABBITMQ_PORT,
        username=settings.RABBITMQ_USERNAME,
        password=settings.RABBITMQ_PASSWORD,
        exchange=settings.RABBITMQ_EXCHANGE
    )
    
    # Connect to services
    await app.state.platform_manager.connect()
    await app.state.storage_manager.connect()
    await app.state.event_publisher.connect()
    
    logger.info("EW service started successfully")
    
    yield
    
    # Shutdown: Clean up resources
    logger.info("Shutting down EW service")
    await app.state.platform_manager.disconnect()
    await app.state.storage_manager.disconnect()
    await app.state.event_publisher.disconnect()
    logger.info("EW service shutdown complete")

# Create FastAPI application
app = FastAPI(
    title="Electronic Warfare Service",
    description="Electronic Warfare service for Bulo.Cloud Sentinel",
    version="0.1.0",
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API router
app.include_router(api_router, prefix="/api/v1")

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    # Using 127.0.0.1 instead of 0.0.0.0 for security - only bind to localhost
    # Use 0.0.0.0 only in development environments or when external access is needed
    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=True)
