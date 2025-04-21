"""
Advanced AI and Analytics Module for Bulo.Cloud Sentinel

This module provides enhanced AI capabilities including object detection,
face recognition, license plate recognition, behavior analysis, and
predictive analytics.
"""

import logging
import os
from fastapi import FastAPI, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware

# Import local modules
from api.routes import detection, recognition, behavior, analytics
from services.video_stream_manager import VideoStreamManager
from services.event_publisher import EventPublisher
from utils.config import load_config

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load configuration
config = load_config()

# Initialize FastAPI app
app = FastAPI(
    title="Bulo.Cloud Sentinel - AI Analytics",
    description="Advanced AI and analytics capabilities for Bulo.Cloud Sentinel",
    version="0.1.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize services
video_manager = VideoStreamManager(config["video_streams"])
event_publisher = EventPublisher(config["event_broker"])

# Store services in app state
app.state.video_manager = video_manager
app.state.event_publisher = event_publisher
app.state.config = config

# Include API routes
app.include_router(detection.router, prefix="/api/detection", tags=["detection"])
app.include_router(recognition.router, prefix="/api/recognition", tags=["recognition"])
app.include_router(behavior.router, prefix="/api/behavior", tags=["behavior"])
app.include_router(analytics.router, prefix="/api/analytics", tags=["analytics"])

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup."""
    logger.info("Starting AI Analytics module")
    
    # Initialize video streams
    await app.state.video_manager.initialize()
    
    # Initialize event publisher
    await app.state.event_publisher.initialize()
    
    logger.info("AI Analytics module started successfully")

@app.on_event("shutdown")
async def shutdown_event():
    """Clean up resources on shutdown."""
    logger.info("Shutting down AI Analytics module")
    
    # Close video streams
    await app.state.video_manager.shutdown()
    
    # Close event publisher
    await app.state.event_publisher.shutdown()
    
    logger.info("AI Analytics module shut down successfully")

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8060, reload=True)
