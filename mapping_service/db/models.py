"""
Database models for the Mapping Service.
"""

import uuid
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from geoalchemy2 import Geography
from sqlalchemy import (
    Boolean, Column, DateTime, Float, ForeignKey, Integer, 
    String, Text, JSON, func
)
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship

Base = declarative_base()


class MappingProject(Base):
    """Mapping project model."""
    
    __tablename__ = "mapping_projects"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(255), nullable=False)
    description = Column(Text, nullable=True)
    created_at = Column(DateTime(timezone=True), default=lambda: datetime.now(timezone.utc))
    updated_at = Column(DateTime(timezone=True), default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))
    user_id = Column(UUID(as_uuid=True), nullable=False)
    status = Column(String(50), nullable=False, default="created")
    image_count = Column(Integer, default=0)
    processing_time = Column(Integer, nullable=True)  # in seconds
    area_coverage = Column(Float, nullable=True)  # in square meters
    resolution = Column(Float, nullable=True)  # in cm/pixel
    boundary = Column(Geography(geometry_type="POLYGON", srid=4326), nullable=True)
    
    # Relationships
    assets = relationship("MappingAsset", back_populates="project", cascade="all, delete-orphan")
    source_images = relationship("SourceImage", back_populates="project", cascade="all, delete-orphan")
    processing_jobs = relationship("ProcessingJob", back_populates="project", cascade="all, delete-orphan")


class MappingAsset(Base):
    """Mapping asset model."""
    
    __tablename__ = "mapping_assets"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    project_id = Column(UUID(as_uuid=True), ForeignKey("mapping_projects.id", ondelete="CASCADE"), nullable=False)
    type = Column(String(50), nullable=False)  # 'orthomosaic', '3d_mesh', 'point_cloud', etc.
    created_at = Column(DateTime(timezone=True), default=lambda: datetime.now(timezone.utc))
    storage_path = Column(String(255), nullable=False)
    file_size = Column(Integer, nullable=True)  # in bytes
    metadata = Column(JSON, nullable=True)
    version = Column(Integer, default=1)
    is_current = Column(Boolean, default=True)
    
    # Relationships
    project = relationship("MappingProject", back_populates="assets")


class SourceImage(Base):
    """Source image model."""
    
    __tablename__ = "source_images"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    project_id = Column(UUID(as_uuid=True), ForeignKey("mapping_projects.id", ondelete="CASCADE"), nullable=False)
    filename = Column(String(255), nullable=False)
    storage_path = Column(String(255), nullable=False)
    captured_at = Column(DateTime(timezone=True), nullable=True)
    location = Column(Geography(geometry_type="POINT", srid=4326), nullable=True)
    altitude = Column(Float, nullable=True)
    heading = Column(Float, nullable=True)
    metadata = Column(JSON, nullable=True)
    
    # Relationships
    project = relationship("MappingProject", back_populates="source_images")


class ProcessingJob(Base):
    """Processing job model."""
    
    __tablename__ = "processing_jobs"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    project_id = Column(UUID(as_uuid=True), ForeignKey("mapping_projects.id", ondelete="CASCADE"), nullable=False)
    status = Column(String(50), nullable=False, default="queued")
    created_at = Column(DateTime(timezone=True), default=lambda: datetime.now(timezone.utc))
    started_at = Column(DateTime(timezone=True), nullable=True)
    completed_at = Column(DateTime(timezone=True), nullable=True)
    error_message = Column(Text, nullable=True)
    parameters = Column(JSON, nullable=True)
    worker_id = Column(String(255), nullable=True)
    
    # Relationships
    project = relationship("MappingProject", back_populates="processing_jobs")
