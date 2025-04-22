"""
Database models for the ISR service.
"""

import uuid
from datetime import datetime
from typing import Dict, List, Optional, Any
from sqlalchemy import Column, String, Float, Integer, Boolean, DateTime, ForeignKey, JSON, Text
from sqlalchemy.orm import relationship
from sqlalchemy.dialects.postgresql import UUID, JSONB

from db.session import Base

class Sensor(Base):
    """Sensor model."""
    __tablename__ = "sensors"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    type = Column(String, nullable=False)  # EO, IR, RADAR, LIDAR, etc.
    platform_id = Column(UUID(as_uuid=True), ForeignKey("platforms.id"), nullable=True)
    status = Column(String, nullable=False, default="offline")  # online, offline, error
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    orientation = Column(JSONB, nullable=True)  # {pitch, roll, yaw}
    capabilities = Column(JSONB, nullable=True)  # Sensor-specific capabilities
    configuration = Column(JSONB, nullable=True)  # Sensor-specific configuration
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    platform = relationship("Platform", back_populates="sensors")
    observations = relationship("Observation", back_populates="sensor", cascade="all, delete-orphan")

class Platform(Base):
    """Platform model (drone, ground station, etc.)."""
    __tablename__ = "platforms"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    type = Column(String, nullable=False)  # drone, ground_station, satellite, etc.
    status = Column(String, nullable=False, default="offline")  # online, offline, error
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    orientation = Column(JSONB, nullable=True)  # {pitch, roll, yaw}
    capabilities = Column(JSONB, nullable=True)  # Platform-specific capabilities
    configuration = Column(JSONB, nullable=True)  # Platform-specific configuration
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    sensors = relationship("Sensor", back_populates="platform")

class Observation(Base):
    """Observation model (raw sensor data)."""
    __tablename__ = "observations"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    sensor_id = Column(UUID(as_uuid=True), ForeignKey("sensors.id"), nullable=False)
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow)
    data_type = Column(String, nullable=False)  # image, point_cloud, radar_return, etc.
    data_url = Column(String, nullable=True)  # URL to raw data in object storage
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    orientation = Column(JSONB, nullable=True)  # {pitch, roll, yaw}
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    sensor = relationship("Sensor", back_populates="observations")
    detections = relationship("Detection", back_populates="observation", cascade="all, delete-orphan")

class Detection(Base):
    """Detection model (processed observation)."""
    __tablename__ = "detections"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    observation_id = Column(UUID(as_uuid=True), ForeignKey("observations.id"), nullable=False)
    target_id = Column(UUID(as_uuid=True), ForeignKey("targets.id"), nullable=True)
    timestamp = Column(DateTime, nullable=False)
    object_type = Column(String, nullable=False)  # person, vehicle, aircraft, etc.
    confidence = Column(Float, nullable=False)
    bounding_box = Column(JSONB, nullable=True)  # {x, y, width, height} or 3D equivalent
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    velocity = Column(JSONB, nullable=True)  # {x, y, z} in m/s
    attributes = Column(JSONB, nullable=True)  # Object-specific attributes
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    observation = relationship("Observation", back_populates="detections")
    target = relationship("Target", back_populates="detections")

class Target(Base):
    """Target model (tracked object)."""
    __tablename__ = "targets"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    track_id = Column(String, nullable=False, unique=True)
    object_type = Column(String, nullable=False)  # person, vehicle, aircraft, etc.
    first_seen = Column(DateTime, nullable=False)
    last_seen = Column(DateTime, nullable=False)
    status = Column(String, nullable=False, default="active")  # active, lost, archived
    confidence = Column(Float, nullable=False)
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    velocity = Column(JSONB, nullable=True)  # {x, y, z} in m/s
    attributes = Column(JSONB, nullable=True)  # Object-specific attributes
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    detections = relationship("Detection", back_populates="target")

class SurveillanceArea(Base):
    """Surveillance area model."""
    __tablename__ = "surveillance_areas"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    description = Column(Text, nullable=True)
    geometry = Column(JSONB, nullable=False)  # GeoJSON geometry
    priority = Column(Integer, nullable=False, default=0)
    active = Column(Boolean, nullable=False, default=True)
    start_time = Column(DateTime, nullable=True)
    end_time = Column(DateTime, nullable=True)
    created_by = Column(String, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class Alert(Base):
    """Alert model."""
    __tablename__ = "alerts"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    type = Column(String, nullable=False)  # intrusion, behavior, system, etc.
    severity = Column(String, nullable=False)  # info, warning, critical
    message = Column(Text, nullable=False)
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow)
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    target_id = Column(UUID(as_uuid=True), ForeignKey("targets.id"), nullable=True)
    surveillance_area_id = Column(UUID(as_uuid=True), ForeignKey("surveillance_areas.id"), nullable=True)
    acknowledged = Column(Boolean, nullable=False, default=False)
    acknowledged_by = Column(String, nullable=True)
    acknowledged_at = Column(DateTime, nullable=True)
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    target = relationship("Target")
    surveillance_area = relationship("SurveillanceArea")
