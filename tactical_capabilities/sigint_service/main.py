"""
SIGINT Service - Signal Intelligence

This module provides the main FastAPI application for the SIGINT service.
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
from services.collector_manager import CollectorManager
from services.signal_analyzer import SignalAnalyzer
from services.direction_finder import DirectionFinder
from services.threat_detector import ThreatDetector
from services.event_publisher import EventPublisher
from services.storage_manager import StorageManager

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Lifespan context manager for startup/shutdown events
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize services and connections
    logger.info("Starting SIGINT service")
    
    # Create database tables if they don't exist
    await create_db_and_tables()
    
    # Initialize services
    app.state.collector_manager = CollectorManager()
    app.state.signal_analyzer = SignalAnalyzer()
    app.state.direction_finder = DirectionFinder()
    app.state.threat_detector = ThreatDetector()
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
    await app.state.collector_manager.connect()
    await app.state.storage_manager.connect()
    await app.state.event_publisher.connect()
    
    logger.info("SIGINT service started successfully")
    
    yield
    
    # Shutdown: Clean up resources
    logger.info("Shutting down SIGINT service")
    await app.state.collector_manager.disconnect()
    await app.state.storage_manager.disconnect()
    await app.state.event_publisher.disconnect()
    logger.info("SIGINT service shutdown complete")

# Create FastAPI application
app = FastAPI(
    title="SIGINT Service",
    description="Signal Intelligence service for Bulo.Cloud Sentinel",
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
