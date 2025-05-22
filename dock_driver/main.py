"""
Main Application for Dock Driver Microservice

This module provides the main FastAPI application for the Dock Driver microservice.
"""

import logging
import os
import yaml
import json
import asyncio
from typing import Dict, Any
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, RedirectResponse
from contextlib import asynccontextmanager
from datetime import timedelta

from dock_driver.api.endpoints import router as api_router
from dock_driver.services.dock_manager import DockManager
from dock_driver.services.power_management import PowerManagementService
from dock_driver.services.auth import authenticate_user, create_access_token, jwt_expiration
from dock_driver.models.schemas import TokenResponse, ErrorResponse

# Configure logging
logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO"),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


# Load configuration
def load_config() -> Dict[str, Any]:
    """
    Load configuration from file.

    Returns:
        Dict[str, Any]: Configuration dictionary.
    """
    try:
        config_path = os.environ.get("DOCK_DRIVER_CONFIG", "config/config.yaml")

        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        # Replace environment variables in config
        config_str = json.dumps(config)
        for key, value in os.environ.items():
            config_str = config_str.replace(f"${{{key}}}", value)

        config = json.loads(config_str)

        return config
    except Exception as e:
        logger.error(f"Error loading configuration: {str(e)}")
        return {}


# Lifespan context manager
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for the application.

    Args:
        app: FastAPI application.
    """
    # Startup
    try:
        # Load configuration
        config = load_config()

        # Initialize dock manager
        dock_manager = DockManager.get_instance()
        success = await dock_manager.initialize()
        if not success:
            logger.error("Failed to initialize dock manager")

        # Initialize power management service
        if config.get("power_management", {}).get("integration_enabled", False):
            power_management = PowerManagementService(config.get("power_management", {}))
            success = await power_management.initialize(dock_manager)
            if not success:
                logger.error("Failed to initialize power management service")

            # Store power management service in app state
            app.state.power_management = power_management

        logger.info("Application started successfully")
    except Exception as e:
        logger.error(f"Error during startup: {str(e)}")

    # Yield control to the application
    yield

    # Shutdown
    try:
        # Shutdown dock manager
        dock_manager = DockManager.get_instance()
        await dock_manager.shutdown()

        # Shutdown power management service
        if hasattr(app.state, "power_management"):
            await app.state.power_management.shutdown()

        logger.info("Application shut down successfully")
    except Exception as e:
        logger.error(f"Error during shutdown: {str(e)}")


# Create FastAPI application
app = FastAPI(
    title="Dock Driver API",
    description="API for controlling and monitoring drone docking stations",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware with secure defaults
# Get allowed origins from environment or config
allowed_origins = os.environ.get("DOCK_DRIVER_ALLOWED_ORIGINS", "http://localhost:3000,http://127.0.0.1:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["Authorization", "Content-Type", "Accept"],
)

# Add middleware to enforce HTTPS
@app.middleware("http")
async def enforce_https(request, call_next):
    # Skip HTTPS enforcement in development
    if os.environ.get("ENVIRONMENT", "development") == "development":
        return await call_next(request)

    # Check if request is secure
    if not request.url.scheme == "https":
        # Redirect to HTTPS
        url = request.url.replace(scheme="https")
        return RedirectResponse(url=str(url), status_code=301)

    return await call_next(request)

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


# Token endpoint
@app.post(
    "/token",
    response_model=TokenResponse,
    responses={
        401: {"model": ErrorResponse, "description": "Invalid credentials"}
    },
    summary="Get access token",
    description="Get an access token for authentication."
)
async def login_for_access_token(form_data: OAuth2PasswordRequestForm = Depends()):
    """
    Get an access token for authentication.

    Args:
        form_data: OAuth2 password request form.

    Returns:
        TokenResponse: Access token response.
    """
    user = await authenticate_user(form_data.username, form_data.password)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token_expires = timedelta(seconds=jwt_expiration)
    access_token = await create_access_token(
        data={"sub": user["username"], "roles": user["roles"]},
        expires_delta=access_token_expires
    )

    return TokenResponse(
        access_token=access_token,
        token_type="bearer",
        expires_in=jwt_expiration
    )


# Health check endpoint
@app.get(
    "/health",
    summary="Health check",
    description="Check the health of the application."
)
async def health_check():
    """
    Check the health of the application.

    Returns:
        Dict[str, Any]: Health check response.
    """
    return {"status": "ok"}


# Include API router
app.include_router(api_router, prefix="/api")


# Exception handler for HTTPException
@app.exception_handler(HTTPException)
async def http_exception_handler(_, exc):
    """
    Handle HTTP exceptions.

    Args:
        _: Request object (unused).
        exc: Exception object.

    Returns:
        JSONResponse: JSON response with error details.
    """
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail},
    )


# Exception handler for other exceptions
@app.exception_handler(Exception)
async def general_exception_handler(_, exc):
    """
    Handle general exceptions.

    Args:
        _: Request object (unused).
        exc: Exception object.

    Returns:
        JSONResponse: JSON response with error details.
    """
    logger.error(f"Unhandled exception: {str(exc)}")
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={"detail": "Internal server error"},
    )


# Run the application
if __name__ == "__main__":
    import uvicorn

    # Load configuration
    config = load_config()
    server_config = config.get("server", {})

    # Get server host and port
    # Default to localhost for security, use environment variable or config for production
    default_host = os.environ.get("DOCK_DRIVER_HOST", "127.0.0.1")
    host = server_config.get("host", default_host)
    port = int(server_config.get("port", 8060))

    # Run the application
    uvicorn.run("main:app", host=host, port=port, reload=server_config.get("debug", False))
