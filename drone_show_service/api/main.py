"""
Drone Show Microservice API

This module provides the FastAPI application for the Drone Show microservice.
"""

import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware

from drone_show_service.api.endpoints import shows, simulation, execution
from drone_show_service.core.config import settings
from drone_show_service.services.choreography_service import ChoreographyService
from drone_show_service.services.simulation_service import SimulationService
from drone_show_service.services.execution_service import ExecutionService
from drone_show_service.services.synchronization_service import SynchronizationService
from drone_show_service.services.led_control_service import LEDControlService
from drone_show_service.services.telemetry_service import TelemetryService
from drone_show_service.services.logging_service import LoggingService
from drone_show_service.services.sentinel_integration import SentinelIntegrationService

# Configure logging
logging.basicConfig(
    level=getattr(logging, settings.LOG_LEVEL),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for the FastAPI application.
    
    This function is called when the application starts and stops.
    It initializes and cleans up resources.
    """
    # Initialize services
    logger.info("Initializing services...")
    
    # Create services
    choreography_service = ChoreographyService()
    simulation_service = SimulationService()
    led_control_service = LEDControlService()
    synchronization_service = SynchronizationService()
    telemetry_service = TelemetryService()
    logging_service = LoggingService()
    sentinel_integration = SentinelIntegrationService(
        api_url=settings.SENTINEL_API_URL,
        api_token=settings.SENTINEL_API_TOKEN,
    )
    
    # Create execution service with dependencies
    execution_service = ExecutionService(
        choreography_service=choreography_service,
        led_control_service=led_control_service,
        synchronization_service=synchronization_service,
        telemetry_service=telemetry_service,
        logging_service=logging_service,
        sentinel_integration=sentinel_integration,
    )
    
    # Store services in app state
    app.state.choreography_service = choreography_service
    app.state.simulation_service = simulation_service
    app.state.execution_service = execution_service
    app.state.led_control_service = led_control_service
    app.state.synchronization_service = synchronization_service
    app.state.telemetry_service = telemetry_service
    app.state.logging_service = logging_service
    app.state.sentinel_integration = sentinel_integration
    
    logger.info("Services initialized")
    
    yield
    
    # Cleanup resources
    logger.info("Cleaning up resources...")
    await execution_service.shutdown()
    logger.info("Resources cleaned up")


# Create FastAPI application
app = FastAPI(
    title="Drone Show Microservice",
    description="API for planning, simulating, and executing drone light shows",
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

# Include routers
app.include_router(shows.router, prefix="/shows", tags=["shows"])
app.include_router(simulation.router, prefix="/simulation", tags=["simulation"])
app.include_router(execution.router, prefix="/execution", tags=["execution"])


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "ok"}


# Dependency to get services from app state
def get_choreography_service(app: FastAPI = Depends()) -> ChoreographyService:
    return app.state.choreography_service


def get_simulation_service(app: FastAPI = Depends()) -> SimulationService:
    return app.state.simulation_service


def get_execution_service(app: FastAPI = Depends()) -> ExecutionService:
    return app.state.execution_service


def get_led_control_service(app: FastAPI = Depends()) -> LEDControlService:
    return app.state.led_control_service


def get_synchronization_service(app: FastAPI = Depends()) -> SynchronizationService:
    return app.state.synchronization_service


def get_telemetry_service(app: FastAPI = Depends()) -> TelemetryService:
    return app.state.telemetry_service


def get_logging_service(app: FastAPI = Depends()) -> LoggingService:
    return app.state.logging_service


def get_sentinel_integration(app: FastAPI = Depends()) -> SentinelIntegrationService:
    return app.state.sentinel_integration
