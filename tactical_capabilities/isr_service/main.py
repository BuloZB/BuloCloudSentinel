"""
ISR Service - Intelligence, Surveillance, and Reconnaissance

This module provides the main FastAPI application for the ISR service.
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
from services.sensor_manager import SensorManager
from services.target_tracker import TargetTracker
from services.data_fusion_engine import DataFusionEngine
from services.event_publisher import EventPublisher

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Lifespan context manager for startup/shutdown events
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize services and connections
    logger.info("Starting ISR service")
    
    # Create database tables if they don't exist
    await create_db_and_tables()
    
    # Initialize services
    app.state.sensor_manager = SensorManager()
    app.state.target_tracker = TargetTracker()
    app.state.data_fusion_engine = DataFusionEngine()
    app.state.event_publisher = EventPublisher(
        host=settings.RABBITMQ_HOST,
        port=settings.RABBITMQ_PORT,
        username=settings.RABBITMQ_USERNAME,
        password=settings.RABBITMQ_PASSWORD,
        exchange=settings.RABBITMQ_EXCHANGE
    )
    
    # Connect to services
    await app.state.sensor_manager.connect()
    await app.state.event_publisher.connect()
    
    logger.info("ISR service started successfully")
    
    yield
    
    # Shutdown: Clean up resources
    logger.info("Shutting down ISR service")
    await app.state.sensor_manager.disconnect()
    await app.state.event_publisher.disconnect()
    logger.info("ISR service shutdown complete")

# Create FastAPI application
app = FastAPI(
    title="ISR Service",
    description="Intelligence, Surveillance, and Reconnaissance service for Bulo.Cloud Sentinel",
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
