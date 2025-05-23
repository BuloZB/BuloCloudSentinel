"""
Main application for the Mapping Service.
"""

import logging
import os
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.openapi.utils import get_openapi

from mapping_service.api.endpoints import router as api_router
from mapping_service.core.config import settings

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan events for the application.
    
    Args:
        app: FastAPI application
    """
    # Startup
    logger.info("Starting Mapping Service")
    
    # Create temporary directory if it doesn't exist
    os.makedirs(settings.TEMP_DIRECTORY, exist_ok=True)
    
    yield
    
    # Shutdown
    logger.info("Shutting down Mapping Service")


app = FastAPI(
    title=settings.PROJECT_NAME,
    openapi_url=f"{settings.API_V1_STR}/openapi.json",
    docs_url=None,
    redoc_url=None,
    lifespan=lifespan,
)

# Set up CORS
if settings.BACKEND_CORS_ORIGINS:
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[str(origin) for origin in settings.BACKEND_CORS_ORIGINS],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

# Include API router
app.include_router(api_router, prefix=settings.API_V1_STR)


@app.get("/docs", include_in_schema=False)
async def custom_swagger_ui_html():
    """
    Custom Swagger UI.
    
    Returns:
        HTML: Swagger UI HTML
    """
    return get_swagger_ui_html(
        openapi_url=f"{settings.API_V1_STR}/openapi.json",
        title=f"{settings.PROJECT_NAME} - Swagger UI",
        oauth2_redirect_url=None,
        swagger_js_url="https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui-bundle.js",
        swagger_css_url="https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui.css",
    )


@app.get("/openapi.json", include_in_schema=False)
async def get_open_api_endpoint():
    """
    Get OpenAPI schema.
    
    Returns:
        dict: OpenAPI schema
    """
    return get_openapi(
        title=settings.PROJECT_NAME,
        version="0.1.0",
        description="API for the Mapping Service",
        routes=app.routes,
    )


@app.get("/")
async def root():
    """
    Root endpoint.
    
    Returns:
        dict: Welcome message
    """
    return {
        "message": "Welcome to the Bulo.Cloud Sentinel Mapping Service",
        "docs": "/docs",
        "version": "0.1.0",
    }
