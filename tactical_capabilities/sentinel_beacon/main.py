"""
SentinelBeacon - Meshtastic-based Mesh Communication Module for Drones

This module provides a robust, decentralized mesh communication network for drones
and ground stations using the Meshtastic protocol.
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
from services.mesh_manager import MeshManager
from services.node_manager import NodeManager
from services.message_manager import MessageManager
from services.channel_manager import ChannelManager
from services.telemetry_manager import TelemetryManager
from services.position_manager import PositionManager
from services.event_publisher import EventPublisher
from services.drone_interface import DroneInterface

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Lifespan context manager for startup/shutdown events
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize services and connections
    logger.info("Starting SentinelBeacon service")
    
    # Create database tables if they don't exist
    await create_db_and_tables()
    
    # Initialize services
    app.state.mesh_manager = MeshManager()
    app.state.node_manager = NodeManager()
    app.state.message_manager = MessageManager()
    app.state.channel_manager = ChannelManager()
    app.state.telemetry_manager = TelemetryManager()
    app.state.position_manager = PositionManager()
    app.state.drone_interface = DroneInterface()
    app.state.event_publisher = EventPublisher(
        host=settings.RABBITMQ_HOST,
        port=settings.RABBITMQ_PORT,
        username=settings.RABBITMQ_USERNAME,
        password=settings.RABBITMQ_PASSWORD,
        exchange=settings.RABBITMQ_EXCHANGE
    )
    
    # Connect to services
    await app.state.mesh_manager.connect()
    await app.state.drone_interface.connect()
    await app.state.event_publisher.connect()
    
    logger.info("SentinelBeacon service started successfully")
    
    yield
    
    # Shutdown: Clean up resources
    logger.info("Shutting down SentinelBeacon service")
    await app.state.mesh_manager.disconnect()
    await app.state.drone_interface.disconnect()
    await app.state.event_publisher.disconnect()
    logger.info("SentinelBeacon service shutdown complete")

# Create FastAPI application
app = FastAPI(
    title="SentinelBeacon",
    description="Meshtastic-based Mesh Communication Module for Drones",
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
