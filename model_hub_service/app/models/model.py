"""
Model schemas for the Model Hub service.

This module provides SQLAlchemy models and Pydantic schemas for the Model Hub service.
"""

import uuid
from datetime import datetime
from typing import Dict, List, Any, Optional, Union
from enum import Enum

from sqlalchemy import Column, String, Float, Boolean, DateTime, Integer, ForeignKey, JSON
from sqlalchemy.orm import relationship

from pydantic import BaseModel, Field, validator

from app.db.database import Base

# SQLAlchemy Models

class ModelStage(str, Enum):
    """Enum for model stages."""
    DEVELOPMENT = "development"
    STAGING = "staging"
    PRODUCTION = "production"
    ARCHIVED = "archived"

class Model(Base):
    """SQLAlchemy model for a machine learning model."""
    __tablename__ = "models"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    name = Column(String, nullable=False, index=True)
    version = Column(String, nullable=False)
    description = Column(String, nullable=True)
    model_type = Column(String, nullable=False)  # e.g., "yolov10", "sam", "super-gradients"
    framework = Column(String, nullable=False)  # e.g., "pytorch", "onnx", "tflite"
    tags = Column(JSON, nullable=True)
    
    # Metadata
    accuracy = Column(Float, nullable=True)
    size_bytes = Column(Integer, nullable=True)
    hash = Column(String, nullable=True)
    signature = Column(String, nullable=True)
    
    # Hardware compatibility
    compatible_hardware = Column(JSON, nullable=True)  # e.g., {"cpu": true, "cuda": true, "jetson": true}
    min_memory_mb = Column(Integer, nullable=True)
    
    # MLflow tracking
    mlflow_run_id = Column(String, nullable=True)
    mlflow_experiment_id = Column(String, nullable=True)
    
    # Status
    stage = Column(String, nullable=False, default=ModelStage.DEVELOPMENT.value)
    is_active = Column(Boolean, nullable=False, default=False)
    
    # Timestamps
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    updated_at = Column(DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    deployments = relationship("Deployment", back_populates="model")

class Deployment(Base):
    """SQLAlchemy model for a model deployment."""
    __tablename__ = "deployments"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    model_id = Column(String, ForeignKey("models.id"), nullable=False)
    environment = Column(String, nullable=False)  # e.g., "production", "staging"
    status = Column(String, nullable=False)  # e.g., "pending", "running", "failed", "completed"
    
    # Deployment details
    deployment_type = Column(String, nullable=False)  # e.g., "blue-green", "canary", "rolling"
    target = Column(String, nullable=False)  # e.g., "edge", "cloud", "all"
    
    # Performance metrics
    fps = Column(Float, nullable=True)
    map = Column(Float, nullable=True)
    latency_ms = Column(Float, nullable=True)
    
    # Rollback details
    previous_deployment_id = Column(String, nullable=True)
    auto_rollback_enabled = Column(Boolean, nullable=False, default=True)
    rollback_threshold = Column(Float, nullable=True)
    
    # Timestamps
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    updated_at = Column(DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)
    completed_at = Column(DateTime, nullable=True)
    
    # Relationships
    model = relationship("Model", back_populates="deployments")

# Pydantic Schemas

class ModelBase(BaseModel):
    """Base schema for a machine learning model."""
    name: str
    version: str
    description: Optional[str] = None
    model_type: str
    framework: str
    tags: Optional[Dict[str, Any]] = None
    
    # Metadata
    accuracy: Optional[float] = None
    size_bytes: Optional[int] = None
    hash: Optional[str] = None
    
    # Hardware compatibility
    compatible_hardware: Optional[Dict[str, bool]] = None
    min_memory_mb: Optional[int] = None

class ModelCreate(ModelBase):
    """Schema for creating a new model."""
    mlflow_run_id: Optional[str] = None
    mlflow_experiment_id: Optional[str] = None
    stage: ModelStage = ModelStage.DEVELOPMENT

class ModelUpdate(BaseModel):
    """Schema for updating a model."""
    name: Optional[str] = None
    description: Optional[str] = None
    tags: Optional[Dict[str, Any]] = None
    accuracy: Optional[float] = None
    stage: Optional[ModelStage] = None
    is_active: Optional[bool] = None

class ModelResponse(ModelBase):
    """Schema for model response."""
    id: str
    stage: ModelStage
    is_active: bool
    created_at: datetime
    updated_at: datetime
    
    class Config:
        from_attributes = True

class DeploymentBase(BaseModel):
    """Base schema for a model deployment."""
    model_id: str
    environment: str
    deployment_type: str
    target: str
    auto_rollback_enabled: bool = True
    rollback_threshold: Optional[float] = None

class DeploymentCreate(DeploymentBase):
    """Schema for creating a new deployment."""
    pass

class DeploymentUpdate(BaseModel):
    """Schema for updating a deployment."""
    status: Optional[str] = None
    fps: Optional[float] = None
    map: Optional[float] = None
    latency_ms: Optional[float] = None
    completed_at: Optional[datetime] = None

class DeploymentResponse(DeploymentBase):
    """Schema for deployment response."""
    id: str
    status: str
    fps: Optional[float] = None
    map: Optional[float] = None
    latency_ms: Optional[float] = None
    previous_deployment_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    completed_at: Optional[datetime] = None
    
    class Config:
        from_attributes = True
