from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging

from backend.api import router as api_router
from backend.api.fleet_management import initialize_service as init_fleet_service
from backend.api.anduril_lattice import initialize_adapter as init_anduril_adapter
from backend.fleet_management.coordinator import FleetCoordinatorService
from backend.mission_planning.service import MissionPlanningService
from backend.mission_execution.execution_service import MissionExecutionService
from dronecore.drone_command_telemetry_hub import DroneCommandHub
from backend.sensor_fusion.engine import SensorFusionEngine
from backend.mesh_networking.mesh_network import MeshNetwork

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

logger = logging.getLogger(__name__)

# Lifespan context manager
from contextlib import asynccontextmanager

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize services
    try:
        logger.info("Initializing services...")

        # Initialize drone command hub
        drone_hub = DroneCommandHub()

        # Initialize sensor fusion engine
        sensor_fusion = SensorFusionEngine()

        # Initialize mesh network
        mesh_network = MeshNetwork()

        # Initialize mission planning service
        mission_service = MissionPlanningService(None)  # Replace None with db_session in production

        # Initialize mission execution service
        execution_service = MissionExecutionService(
            mission_service=mission_service,
            drone_command_hub=drone_hub
        )

        # Initialize fleet coordinator service
        fleet_service = FleetCoordinatorService(
            mission_service=mission_service,
            execution_service=execution_service,
            drone_command_hub=drone_hub
        )

        # Initialize Anduril Lattice adapter
        init_anduril_adapter(sensor_fusion, mesh_network)

        # Initialize fleet management service
        init_fleet_service(fleet_service)

        logger.info("Services initialized successfully")
    except Exception as e:
        logger.error(f"Error initializing services: {str(e)}")

    yield

    # Shutdown: Clean up resources
    logger.info("Shutting down services...")

# Create FastAPI app
app = FastAPI(title="Bulo.Cloud Sentinel Backend API", lifespan=lifespan)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API router
app.include_router(api_router, prefix="/api")


