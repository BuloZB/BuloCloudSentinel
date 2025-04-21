"""
SentinelWeb Backend - Main Application

This is the main entry point for the SentinelWeb backend service.
It initializes the FastAPI application, sets up middleware, and includes routers.
"""

import logging
from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from backend.core.config import settings
from backend.api.router import api_router
from backend.core.auth import get_current_user
from backend.services.sentinel_client import SentinelClient
from backend.db.session import create_db_and_tables, get_db_session

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Lifespan context manager for startup/shutdown events
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize services and connections
    logger.info("Starting SentinelWeb backend service")
    
    # Create database tables if they don't exist
    await create_db_and_tables()
    
    # Initialize connection to BuloCloudSentinel
    sentinel_client = SentinelClient(
        base_url=settings.SENTINEL_API_URL,
        token=settings.SENTINEL_API_TOKEN
    )
    app.state.sentinel_client = sentinel_client
    
    # Check connection to BuloCloudSentinel
    try:
        health = await sentinel_client.check_health()
        logger.info(f"Connected to BuloCloudSentinel: {health}")
    except Exception as e:
        logger.warning(f"Could not connect to BuloCloudSentinel: {str(e)}")
    
    yield
    
    # Shutdown: Clean up resources
    logger.info("Shutting down SentinelWeb backend service")

# Create FastAPI app
app = FastAPI(
    title="SentinelWeb API",
    description="API for SentinelWeb, a web interface addon for BuloCloudSentinel",
    version="0.1.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API router
app.include_router(api_router, prefix="/api")

# Root endpoint
@app.get("/")
async def root():
    return {
        "name": "SentinelWeb API",
        "version": "0.1.0",
        "status": "running"
    }

# Health check endpoint
@app.get("/health")
async def health_check():
    return {"status": "healthy"}

# Protected endpoint example
@app.get("/protected")
async def protected_route(current_user = Depends(get_current_user)):
    return {"message": f"Hello, {current_user.username}!"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
