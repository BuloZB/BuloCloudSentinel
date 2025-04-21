"""
Drone Swarm System for Bulo.Cloud Sentinel

This module provides advanced multi-drone coordination and autonomous mission planning.
"""

import logging
import os
from fastapi import FastAPI, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware

# Import local modules
from api.routes import drones, missions, swarm, geofencing, weather
from services.drone_service import DroneService
from services.mission_service import MissionService
from services.telemetry_service import TelemetryService
from coordinator.swarm_manager import SwarmManager
from planning.mission_planner import MissionPlanner
from safety.collision_avoider import CollisionAvoider
from safety.weather_monitor import WeatherMonitor
from safety.power_manager import PowerManager
from utils.config import load_config

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load configuration
config = load_config()

# Initialize FastAPI app
app = FastAPI(
    title="Bulo.Cloud Sentinel - Drone Swarm System",
    description="Advanced multi-drone coordination and autonomous mission planning",
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
drone_service = DroneService(config["drones"])
mission_service = MissionService(config["missions"])
telemetry_service = TelemetryService(config["telemetry"])

# Initialize coordinators
swarm_manager = SwarmManager(config["swarm"], drone_service, telemetry_service)
mission_planner = MissionPlanner(config["planning"], drone_service)
collision_avoider = CollisionAvoider(config["safety"]["collision_avoidance"])
weather_monitor = WeatherMonitor(config["safety"]["weather"])
power_manager = PowerManager(config["safety"]["power"])

# Store services in app state
app.state.drone_service = drone_service
app.state.mission_service = mission_service
app.state.telemetry_service = telemetry_service
app.state.swarm_manager = swarm_manager
app.state.mission_planner = mission_planner
app.state.collision_avoider = collision_avoider
app.state.weather_monitor = weather_monitor
app.state.power_manager = power_manager
app.state.config = config

# Include API routes
app.include_router(drones.router, prefix="/api/drones", tags=["drones"])
app.include_router(missions.router, prefix="/api/missions", tags=["missions"])
app.include_router(swarm.router, prefix="/api/swarm", tags=["swarm"])
app.include_router(geofencing.router, prefix="/api/geofencing", tags=["geofencing"])
app.include_router(weather.router, prefix="/api/weather", tags=["weather"])

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup."""
    logger.info("Starting Drone Swarm System")
    
    # Initialize drone service
    await app.state.drone_service.initialize()
    
    # Initialize mission service
    await app.state.mission_service.initialize()
    
    # Initialize telemetry service
    await app.state.telemetry_service.initialize()
    
    # Initialize swarm manager
    await app.state.swarm_manager.initialize()
    
    # Initialize weather monitor
    await app.state.weather_monitor.initialize()
    
    logger.info("Drone Swarm System started successfully")

@app.on_event("shutdown")
async def shutdown_event():
    """Clean up resources on shutdown."""
    logger.info("Shutting down Drone Swarm System")
    
    # Shutdown services
    await app.state.drone_service.shutdown()
    await app.state.mission_service.shutdown()
    await app.state.telemetry_service.shutdown()
    await app.state.swarm_manager.shutdown()
    await app.state.weather_monitor.shutdown()
    
    logger.info("Drone Swarm System shut down successfully")

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8070, reload=True)
