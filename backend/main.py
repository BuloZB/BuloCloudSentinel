"""
SentinelWeb Backend - Main Application

This is the main entry point for the SentinelWeb backend service.
It initializes the FastAPI application, sets up middleware, and includes routers.
"""

import logging
from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.middleware.gzip import GZipMiddleware
from starlette.middleware.sessions import SessionMiddleware
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

# Add security middlewares
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "PATCH"],
    allow_headers=["Authorization", "Content-Type", "Accept", "Origin", "X-Requested-With"],
    expose_headers=["X-Total-Count", "Content-Disposition"]
)

# Add trusted host middleware
app.add_middleware(
    TrustedHostMiddleware, allowed_hosts=["*"]  # In production, specify actual hostnames
)

# Add session middleware with secure settings
app.add_middleware(
    SessionMiddleware,
    secret_key=settings.JWT_SECRET,
    max_age=settings.JWT_EXPIRATION_MINUTES * 60,
    https_only=True,
    same_site="lax"
)

# Add compression middleware
app.add_middleware(GZipMiddleware, minimum_size=1000)

# Import security modules
from security.middleware.security_middleware import SecurityHeadersMiddleware, RateLimitingMiddleware
from security.middleware.csrf import CSRFMiddleware
from security.middleware.csp import ContentSecurityPolicyMiddleware, get_default_csp_policy
from security.utils.secure_logging import setup_secure_logger
from security.utils.encryption import EncryptionService

# Add security headers middleware
app.add_middleware(
    SecurityHeadersMiddleware,
    content_security_policy="default-src 'self'; script-src 'self'; frame-ancestors 'none'",
    permissions_policy="camera=(), microphone=(), geolocation=()"
)

# Add CSRF middleware for non-API routes
app.add_middleware(
    CSRFMiddleware,
    secret_key=settings.JWT_SECRET,
    exclude_paths=["/api/", "/docs", "/redoc", "/openapi.json"]
)

# Add rate limiting middleware
app.add_middleware(
    RateLimitingMiddleware,
    rate_limit=100,  # 100 requests per minute
    rate_limit_window=60,  # 1 minute window
    exclude_paths=["/docs", "/redoc", "/openapi.json"]
)

# Add CSP middleware
app.add_middleware(
    ContentSecurityPolicyMiddleware,
    policy=get_default_csp_policy()
)

# Set up secure logger
logger = setup_secure_logger("bulo_cloud_sentinel")
app.state.logger = logger

# Set up encryption service
encryption_service = EncryptionService(master_key=settings.JWT_SECRET)
app.state.encryption_service = encryption_service

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
