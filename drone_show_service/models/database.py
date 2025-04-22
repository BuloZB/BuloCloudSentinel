"""
Database models for the Drone Show microservice.

This module provides SQLAlchemy models for storing choreography data in the database.
"""

import uuid
from datetime import datetime
from typing import Dict, Any, List, Optional

from sqlalchemy import Column, String, Integer, Float, Boolean, DateTime, ForeignKey, JSON, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship

Base = declarative_base()


class ChoreographyDB(Base):
    """Database model for choreographies."""
    __tablename__ = "choreographies"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    name = Column(String, nullable=False)
    description = Column(Text, nullable=True)
    author = Column(String, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    tags = Column(JSON, default=list)
    duration = Column(Float, nullable=False)
    drone_count = Column(Integer, nullable=False)
    status = Column(String, nullable=False, default="draft")
    type = Column(String, nullable=False)
    trajectories = Column(JSON, nullable=False)
    formations = Column(JSON, nullable=True)
    music_file = Column(String, nullable=True)
    music_bpm = Column(Float, nullable=True)
    music_offset = Column(Float, nullable=True)
    boundary = Column(JSON, nullable=True)
    home_position = Column(JSON, nullable=True)
    notes = Column(Text, nullable=True)
    
    # Relationships
    simulations = relationship("SimulationDB", back_populates="choreography", cascade="all, delete-orphan")
    executions = relationship("ExecutionDB", back_populates="choreography", cascade="all, delete-orphan")


class SimulationDB(Base):
    """Database model for simulations."""
    __tablename__ = "simulations"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    choreography_id = Column(String, ForeignKey("choreographies.id"), nullable=False)
    settings = Column(JSON, nullable=False)
    frames_file = Column(String, nullable=False)  # Path to frames file in MinIO
    duration = Column(Float, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    choreography = relationship("ChoreographyDB", back_populates="simulations")


class ExecutionDB(Base):
    """Database model for executions."""
    __tablename__ = "executions"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    choreography_id = Column(String, ForeignKey("choreographies.id"), nullable=False)
    settings = Column(JSON, nullable=False)
    status = Column(String, nullable=False, default="pending")
    drone_statuses = Column(JSON, nullable=False)
    start_time = Column(DateTime, nullable=True)
    end_time = Column(DateTime, nullable=True)
    current_time = Column(Float, default=0.0)
    progress = Column(Float, default=0.0)
    error_message = Column(Text, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    choreography = relationship("ChoreographyDB", back_populates="executions")
    logs = relationship("ExecutionLogDB", back_populates="execution", cascade="all, delete-orphan")


class ExecutionLogDB(Base):
    """Database model for execution logs."""
    __tablename__ = "execution_logs"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    execution_id = Column(String, ForeignKey("executions.id"), nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow)
    level = Column(String, nullable=False)
    message = Column(Text, nullable=False)
    data = Column(JSON, nullable=True)
    
    # Relationships
    execution = relationship("ExecutionDB", back_populates="logs")
