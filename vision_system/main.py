"""
Vision System for Crowd and Vehicle Analysis

This microservice provides advanced computer vision capabilities for analyzing
aerial footage from drones, with a focus on crowd density estimation and vehicle tracking.
"""

import logging
import os
from fastapi import FastAPI, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware

# Import local modules
from api.routes import analysis, streams, calibration, visualization
from services.analysis_service import AnalysisService
from services.stream_service import StreamService
from services.storage_service import StorageService
from processors.crowd_analyzer import CrowdAnalyzer
from processors.vehicle_analyzer import VehicleAnalyzer
from processors.flow_analyzer import FlowAnalyzer
from processors.heat_mapper import HeatMapper
from utils.config import load_config

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load configuration
config = load_config()

# Initialize FastAPI app
app = FastAPI(
    title="Bulo.Cloud Sentinel - Vision System",
    description="Advanced computer vision for crowd density and vehicle analysis",
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
storage_service = StorageService(config["storage"])
stream_service = StreamService(config["streams"], storage_service)

# Initialize processors
crowd_analyzer = CrowdAnalyzer(config["models"]["crowd"])
vehicle_analyzer = VehicleAnalyzer(config["models"]["vehicle"])
flow_analyzer = FlowAnalyzer(config["models"]["tracking"])
heat_mapper = HeatMapper(config["visualization"])

# Initialize analysis service
analysis_service = AnalysisService(
    config["analysis"],
    stream_service,
    storage_service,
    crowd_analyzer,
    vehicle_analyzer,
    flow_analyzer,
    heat_mapper
)

# Store services in app state
app.state.analysis_service = analysis_service
app.state.stream_service = stream_service
app.state.storage_service = storage_service
app.state.config = config

# Include API routes
app.include_router(analysis.router, prefix="/api/analysis", tags=["analysis"])
app.include_router(streams.router, prefix="/api/streams", tags=["streams"])
app.include_router(calibration.router, prefix="/api/calibration", tags=["calibration"])
app.include_router(visualization.router, prefix="/api/visualization", tags=["visualization"])

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup."""
    logger.info("Starting Vision System")

    # Initialize storage service
    await app.state.storage_service.initialize()

    # Initialize stream service
    await app.state.stream_service.initialize()

    # Initialize analysis service
    await app.state.analysis_service.initialize()

    logger.info("Vision System started successfully")

@app.on_event("shutdown")
async def shutdown_event():
    """Clean up resources on shutdown."""
    logger.info("Shutting down Vision System")

    # Shutdown services
    await app.state.analysis_service.shutdown()
    await app.state.stream_service.shutdown()
    await app.state.storage_service.shutdown()

    logger.info("Vision System shut down successfully")

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    # Using 127.0.0.1 instead of 0.0.0.0 for security - only bind to localhost
    # Use 0.0.0.0 only in development environments where external access is needed
    uvicorn.run("main:app", host="127.0.0.1", port=8080, reload=True)
