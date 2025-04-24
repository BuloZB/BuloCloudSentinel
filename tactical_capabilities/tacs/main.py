"""
Main application module for the TACS service.
"""

import logging
from fastapi import FastAPI, Depends, Request, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import time
from typing import Callable

from api.router import api_router
from core.config import settings
from core.security import get_current_user
from db.session import get_db_session, init_db

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Target Acquisition and Coordination System",
    description="Advanced sensor fusion and coordination module for Bulo.Cloud Sentinel",
    version="0.1.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add request ID middleware
@app.middleware("http")
async def add_request_id(request: Request, call_next: Callable) -> Response:
    """Add request ID to response headers."""
    import uuid
    request_id = str(uuid.uuid4())
    response = await call_next(request)
    response.headers["X-Request-ID"] = request_id
    return response

# Add request logging middleware
@app.middleware("http")
async def log_requests(request: Request, call_next: Callable) -> Response:
    """Log request information."""
    start_time = time.time()
    response = await call_next(request)
    process_time = time.time() - start_time
    logger.info(
        f"{request.method} {request.url.path} {response.status_code} {process_time:.4f}s"
    )
    return response

# Add rate limiting middleware
@app.middleware("http")
async def rate_limit(request: Request, call_next: Callable) -> Response:
    """Apply rate limiting."""
    # In a real implementation, this would use Redis or another distributed cache
    # For now, we'll just pass through
    return await call_next(request)

# Add security headers middleware
@app.middleware("http")
async def add_security_headers(request: Request, call_next: Callable) -> Response:
    """Add security headers to responses."""
    response = await call_next(request)
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["X-XSS-Protection"] = "1; mode=block"
    response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
    response.headers["Content-Security-Policy"] = "default-src 'self'"
    return response

# Include API router
app.include_router(api_router, prefix="/api/v1")

# Health check endpoint
@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "ok", "service": "tacs"}

# Readiness check endpoint
@app.get("/ready")
async def readiness_check():
    """Readiness check endpoint."""
    return {"status": "ready", "service": "tacs"}

# Error handlers
@app.exception_handler(Exception)
async def generic_exception_handler(request: Request, exc: Exception):
    """Handle generic exceptions."""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error"},
    )

# Startup event
@app.on_event("startup")
async def startup_event():
    """Run startup tasks."""
    logger.info("Starting TACS service")
    
    # Initialize database
    await init_db()
    
    logger.info("TACS service started")

# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Run shutdown tasks."""
    logger.info("Shutting down TACS service")
    
    # Cleanup resources
    
    logger.info("TACS service shut down")
