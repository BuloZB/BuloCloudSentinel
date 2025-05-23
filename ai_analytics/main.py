"""
Advanced AI and Analytics Module for Bulo.Cloud Sentinel

This module provides enhanced AI capabilities including object detection,
face recognition, license plate recognition, behavior analysis, and
predictive analytics.
"""

import logging
import os
import sys
from fastapi import FastAPI, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import local modules
from api.routes import detection, recognition, behavior, analytics, multimodal, inference
from services.video_stream_manager import VideoStreamManager
from services.event_publisher import EventPublisher
from services.multimodal_detection import MultimodalDetectionService
from services.inference_service import InferenceService
from services.enhanced_detection import EnhancedDetectionService
from utils.config import load_config, get_config
from api.dependencies import (
    get_video_stream_manager,
    get_event_publisher,
    get_detection_service,
    get_recognition_service,
    get_behavior_service,
    get_analytics_service,
    get_multimodal_detection_service
)

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

# Initialize inference service
inference_service = InferenceService(config["inference"])

# Store services in app state
app.state.video_manager = video_manager
app.state.event_publisher = event_publisher
app.state.inference_service = inference_service
app.state.config = config

# Include API routes
app.include_router(detection.router, prefix="/api/detection", tags=["detection"])
app.include_router(recognition.router, prefix="/api/recognition", tags=["recognition"])
app.include_router(behavior.router, prefix="/api/behavior", tags=["behavior"])
app.include_router(analytics.router, prefix="/api/analytics", tags=["analytics"])
app.include_router(multimodal.router, prefix="/api/multimodal", tags=["multimodal"])
app.include_router(inference.router, prefix="/api/inference", tags=["inference"])

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
    # Using 127.0.0.1 instead of 0.0.0.0 for security - only bind to localhost
    # Use 0.0.0.0 only in development environments where external access is needed
    uvicorn.run("main:app", host="127.0.0.1", port=8060, reload=True)
