"""
Main application module for Weather Guard service.

This module provides the main FastAPI application for the Weather Guard service.
"""

import logging
import sys
from contextlib import asynccontextmanager
from typing import List

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.base import BaseHTTPMiddleware

from weather_guard import __version__
from weather_guard.api.router import api_router
from weather_guard.core.config import settings
from weather_guard.services.weather import WeatherService
from weather_guard.services.alerts import AlertService
from weather_guard.services.metrics import MetricsExporter

# Configure logging
logging.basicConfig(
    level=getattr(logging, settings.LOG_LEVEL),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)

logger = logging.getLogger(__name__)


class RequestLoggingMiddleware(BaseHTTPMiddleware):
    """Middleware for logging requests."""

    async def dispatch(self, request, call_next):
        """Log request and response."""
        logger.debug(f"Request: {request.method} {request.url.path}")
        response = await call_next(request)
        logger.debug(f"Response: {response.status_code}")
        return response


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for the FastAPI application.

    This function is called when the application starts and stops.
    It initializes and cleans up resources.
    """
    # Initialize services
    logger.info("Initializing services...")

    # Create weather service
    weather_service = WeatherService()
    await weather_service.initialize()

    # Create alert service
    alert_service = AlertService(weather_service)
    await alert_service.start()

    # Create metrics exporter
    metrics_exporter = MetricsExporter(weather_service)
    metrics_exporter.start()

    # Store in app state
    app.state.weather_service = weather_service
    app.state.alert_service = alert_service
    app.state.metrics_exporter = metrics_exporter

    logger.info("Application startup complete")
    yield

    # Cleanup resources
    logger.info("Shutting down services...")
    await alert_service.stop()
    await metrics_exporter.stop()
    await weather_service.close()

    logger.info("Application shutdown complete")


# Create FastAPI application
app = FastAPI(
    title=settings.PROJECT_NAME,
    description="Weather awareness service for Bulo.Cloud Sentinel",
    version=__version__,
    lifespan=lifespan,
)

# Add middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[str(origin) for origin in settings.BACKEND_CORS_ORIGINS],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.add_middleware(RequestLoggingMiddleware)

# Include API router
app.include_router(api_router, prefix=settings.API_V1_STR)


@app.get("/")
async def root():
    """Root endpoint for the API."""
    return {
        "name": settings.PROJECT_NAME,
        "version": __version__,
        "description": "Weather awareness service for Bulo.Cloud Sentinel",
        "docs_url": "/docs",
    }


@app.get("/health")
async def health():
    """Health check endpoint."""
    return {"status": "ok", "version": __version__}


if __name__ == "__main__":
    import uvicorn

    # Run the application
    uvicorn.run(
        "weather_guard.main:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=True,
    )
