#!/usr/bin/env python3
"""
Bulo.CloudSentinel Model Hub Service

This module provides a FastAPI application for the Model Hub service.
"""

import os
import sys
import logging
from typing import Dict, List, Any, Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Import routers
from app.api.v1.models import router as models_router
from app.api.v1.deployments import router as deployments_router
from app.api.v1.health import router as health_router
from app.api.v1.grpc import router as grpc_router
from app.api.v1.training import router as training_router
from app.api.v1.ab_testing import router as ab_testing_router
from app.api.v1.visualizations import router as visualizations_router
from app.api.v1.federated_learning import router as federated_learning_router
from app.api.v1.explainability import router as explainability_router
from app.api.v1.optimization import router as optimization_router

# Import services
from app.services.mlflow_service import MLflowService
from app.services.deployment_service import DeploymentService
from app.services.security_service import SecurityService
from app.services.git_lfs_service import GitLFSService
from app.services.training_service import TrainingService

# Import database
from app.db.database import init_db, get_db

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for the FastAPI application.

    This function is called when the application starts and stops.
    It initializes the database and services.
    """
    # Initialize database
    logger.info("Initializing database...")
    await init_db()

    # Initialize MLflow service
    logger.info("Initializing MLflow service...")
    mlflow_service = MLflowService()
    app.state.mlflow_service = mlflow_service

    # Initialize deployment service
    logger.info("Initializing deployment service...")
    deployment_service = DeploymentService()
    app.state.deployment_service = deployment_service

    # Initialize security service
    logger.info("Initializing security service...")
    security_service = SecurityService()
    app.state.security_service = security_service

    # Initialize Git LFS service
    logger.info("Initializing Git LFS service...")
    git_lfs_service = GitLFSService()
    app.state.git_lfs_service = git_lfs_service

    # Initialize training service
    logger.info("Initializing training service...")
    training_service = TrainingService()
    app.state.training_service = training_service

    logger.info("Application startup complete")
    yield
    logger.info("Application shutdown")

# Create FastAPI application
app = FastAPI(
    title="Bulo.CloudSentinel Model Hub",
    description="API for the Bulo.CloudSentinel Model Hub service",
    version="0.1.0",
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health_router, prefix="/api/v1/health", tags=["health"])
app.include_router(models_router, prefix="/api/v1/models", tags=["models"])
app.include_router(deployments_router, prefix="/api/v1/deployments", tags=["deployments"])
app.include_router(grpc_router, prefix="/api/v1/grpc", tags=["grpc"])
app.include_router(training_router, prefix="/api/v1/training", tags=["training"])
app.include_router(ab_testing_router, prefix="/api/v1/ab-testing", tags=["ab-testing"])
app.include_router(visualizations_router, prefix="/api/v1/visualizations", tags=["visualizations"])
app.include_router(federated_learning_router, prefix="/api/v1/federated-learning", tags=["federated-learning"])
app.include_router(explainability_router, prefix="/api/v1/explainability", tags=["explainability"])
app.include_router(optimization_router, prefix="/api/v1/optimization", tags=["optimization"])

@app.get("/", tags=["root"])
async def root():
    """Root endpoint for the API."""
    return {
        "message": "Welcome to the Bulo.CloudSentinel Model Hub API",
        "docs_url": "/docs",
    }

if __name__ == "__main__":
    import uvicorn

    # Get port from environment variable or use default
    port = int(os.environ.get("PORT", 8070))

    # Run the application
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=port,
        reload=True,
    )
