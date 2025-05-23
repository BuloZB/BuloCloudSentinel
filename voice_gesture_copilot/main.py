"""
Voice & Gesture Co-Pilot Module for Bulo.Cloud Sentinel

This module provides hands-free drone control through voice commands and hand gestures,
enhancing operational efficiency in field conditions.
"""

import os
import logging
import asyncio
from contextlib import asynccontextmanager
from typing import Dict, Any, List, Optional

import uvicorn
from fastapi import FastAPI, Depends, HTTPException, status, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from voice_gesture_copilot.api.routes import voice_routes, gesture_routes, command_routes, config_routes
from voice_gesture_copilot.core.config import settings
from voice_gesture_copilot.core.logging import setup_logging
from voice_gesture_copilot.services.recognition_service import RecognitionService
from voice_gesture_copilot.services.command_service import CommandService
from voice_gesture_copilot.services.drone_service import DroneService
from voice_gesture_copilot.services.feedback_service import FeedbackService

# Setup logging
setup_logging()
logger = logging.getLogger(__name__)

# Application lifespan context manager
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for the FastAPI application.
    Initializes services on startup and cleans up on shutdown.
    """
    # Initialize services
    logger.info("Initializing Voice & Gesture Co-Pilot services")
    
    # Get configuration from environment variables
    sentinel_api_url = os.environ.get("SENTINEL_API_URL", "http://bulocloud-sentinel-api:8000")
    sentinel_api_token = os.environ.get("SENTINEL_API_TOKEN", "")
    
    # Initialize services
    app.state.drone_service = DroneService(sentinel_api_url, sentinel_api_token)
    app.state.recognition_service = RecognitionService()
    app.state.command_service = CommandService(app.state.drone_service)
    app.state.feedback_service = FeedbackService()
    
    # Load models
    await app.state.recognition_service.load_models()
    await app.state.command_service.load_models()
    
    logger.info("Voice & Gesture Co-Pilot services initialized")
    
    yield
    
    # Cleanup resources
    logger.info("Shutting down Voice & Gesture Co-Pilot services")
    await app.state.recognition_service.cleanup()
    await app.state.command_service.cleanup()
    await app.state.drone_service.cleanup()
    await app.state.feedback_service.cleanup()

# Create FastAPI application
app = FastAPI(
    title="Voice & Gesture Co-Pilot",
    description="Hands-free drone control through voice commands and hand gestures",
    version="1.0.0",
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

# Include API routers
app.include_router(voice_routes.router, prefix="/api/voice", tags=["voice"])
app.include_router(gesture_routes.router, prefix="/api/gesture", tags=["gesture"])
app.include_router(command_routes.router, prefix="/api/command", tags=["command"])
app.include_router(config_routes.router, prefix="/api/config", tags=["config"])

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: Dict[str, Any]):
        for connection in self.active_connections:
            await connection.send_json(message)

manager = ConnectionManager()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            # Wait for messages from the client
            data = await websocket.receive_json()
            
            # Process the message based on its type
            if data.get("type") == "voice_command":
                # Process voice command
                command_text = data.get("text", "")
                if command_text:
                    result = await app.state.command_service.process_voice_command(command_text)
                    await websocket.send_json(result)
            
            elif data.get("type") == "gesture":
                # Process gesture
                gesture_data = data.get("data", {})
                if gesture_data:
                    result = await app.state.command_service.process_gesture(gesture_data)
                    await websocket.send_json(result)
    
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# Mount static files
app.mount("/static", StaticFiles(directory="voice_gesture_copilot/static"), name="static")

# Root endpoint
@app.get("/", tags=["root"])
async def root():
    """Root endpoint that returns basic module information."""
    return {
        "name": "Voice & Gesture Co-Pilot",
        "version": "1.0.0",
        "description": "Hands-free drone control through voice commands and hand gestures",
        "status": "active",
    }

# Health check endpoint
@app.get("/health", tags=["health"])
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "services": {
            "recognition": await app.state.recognition_service.status(),
            "command": await app.state.command_service.status(),
            "drone": await app.state.drone_service.status(),
            "feedback": await app.state.feedback_service.status(),
        }
    }

if __name__ == "__main__":
    uvicorn.run(
        "voice_gesture_copilot.main:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=settings.DEBUG,
    )
