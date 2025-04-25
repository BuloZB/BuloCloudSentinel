"""
Database models for the TACS module.
"""

import uuid
from datetime import datetime, timezone
from typing import Dict, List, Optional, Any
from sqlalchemy import Column, String, Integer, Float, Boolean, DateTime, ForeignKey, JSON, Table
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.orm import relationship
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

# Association tables
target_sensor_association = Table(
    'target_sensor_association',
    Base.metadata,
    Column('target_id', UUID(as_uuid=True), ForeignKey('targets.id')),
    Column('sensor_id', UUID(as_uuid=True), ForeignKey('sensors.id'))
)

plan_target_association = Table(
    'plan_target_association',
    Base.metadata,
    Column('plan_id', UUID(as_uuid=True), ForeignKey('coordination_plans.id')),
    Column('target_id', UUID(as_uuid=True), ForeignKey('targets.id'))
)

plan_platform_association = Table(
    'plan_platform_association',
    Base.metadata,
    Column('plan_id', UUID(as_uuid=True), ForeignKey('coordination_plans.id')),
    Column('platform_id', UUID(as_uuid=True))
)

class Target(Base):
    """Target database model."""
    __tablename__ = "targets"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    type = Column(String, nullable=False)
    classification = Column(String)
    confidence = Column(Float, nullable=False)
    location = Column(JSONB, nullable=False)
    velocity = Column(JSONB)
    dimensions = Column(JSONB)
    first_detected = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))
    last_updated = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))
    metadata = Column(JSONB, default={})
    priority = Column(Integer, default=1)
    status = Column(String, nullable=False, default="active")
    created_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))
    updated_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))

    # Relationships
    tracks = relationship("Track", back_populates="target", cascade="all, delete-orphan")
    sensors = relationship("Sensor", secondary=target_sensor_association, back_populates="targets")
    plans = relationship("CoordinationPlan", secondary=plan_target_association, back_populates="targets")

class TrackPoint(Base):
    """Track point database model."""
    __tablename__ = "track_points"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    track_id = Column(UUID(as_uuid=True), ForeignKey("tracks.id"), nullable=False)
    timestamp = Column(DateTime, nullable=False)
    location = Column(JSONB, nullable=False)
    altitude = Column(Float)
    velocity = Column(JSONB)
    acceleration = Column(JSONB)
    heading = Column(Float)
    confidence = Column(Float, nullable=False)
    sensor_id = Column(UUID(as_uuid=True), ForeignKey("sensors.id"), nullable=False)
    created_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))

    # Relationships
    track = relationship("Track", back_populates="points")
    sensor = relationship("Sensor")

class Track(Base):
    """Track database model."""
    __tablename__ = "tracks"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    target_id = Column(UUID(as_uuid=True), ForeignKey("targets.id"), nullable=False)
    start_time = Column(DateTime, nullable=False)
    end_time = Column(DateTime)
    quality = Column(Float, nullable=False)
    status = Column(String, nullable=False, default="active")
    metadata = Column(JSONB, default={})
    created_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))
    updated_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))

    # Relationships
    target = relationship("Target", back_populates="tracks")
    points = relationship("TrackPoint", back_populates="track", cascade="all, delete-orphan", order_by="TrackPoint.timestamp")

class Sensor(Base):
    """Sensor database model."""
    __tablename__ = "sensors"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    type = Column(String, nullable=False)
    platform_id = Column(UUID(as_uuid=True), nullable=False)
    capabilities = Column(JSONB, default=[])
    resolution = Column(JSONB)
    field_of_view = Column(JSONB)
    range = Column(Float)
    accuracy = Column(Float)
    status = Column(String, nullable=False, default="online")
    metadata = Column(JSONB, default={})
    created_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))
    updated_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))

    # Relationships
    targets = relationship("Target", secondary=target_sensor_association, back_populates="sensors")
    sensor_data = relationship("SensorData", back_populates="sensor", cascade="all, delete-orphan")

class SensorData(Base):
    """Sensor data database model."""
    __tablename__ = "sensor_data"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    sensor_id = Column(UUID(as_uuid=True), ForeignKey("sensors.id"), nullable=False)
    timestamp = Column(DateTime, nullable=False)
    data_type = Column(String, nullable=False)
    data_url = Column(String)
    metadata = Column(JSONB, default={})
    created_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))

    # Relationships
    sensor = relationship("Sensor", back_populates="sensor_data")
    fusion_jobs = relationship("FusionJob", secondary="fusion_job_sensor_data", back_populates="sensor_data")

class CoordinationPlan(Base):
    """Coordination plan database model."""
    __tablename__ = "coordination_plans"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    start_time = Column(DateTime, nullable=False)
    end_time = Column(DateTime)
    waypoints = Column(JSONB, default={})
    sensor_configurations = Column(JSONB, default={})
    priority = Column(Integer, default=1)
    status = Column(String, nullable=False, default="draft")
    metadata = Column(JSONB, default={})
    created_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))
    updated_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))

    # Relationships
    targets = relationship("Target", secondary=plan_target_association, back_populates="plans")

class FusionJob(Base):
    """Fusion job database model."""
    __tablename__ = "fusion_jobs"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    target_id = Column(UUID(as_uuid=True), ForeignKey("targets.id"))
    status = Column(String, nullable=False, default="pending")
    parameters = Column(JSONB, default={})
    result_id = Column(UUID(as_uuid=True))
    error_message = Column(String)
    priority = Column(Integer, default=1)
    metadata = Column(JSONB, default={})
    created_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))
    updated_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))
    completed_at = Column(DateTime)

    # Relationships
    target = relationship("Target")
    sensor_data = relationship("SensorData", secondary="fusion_job_sensor_data", back_populates="fusion_jobs")

# Association table for fusion jobs and sensor data
fusion_job_sensor_data = Table(
    'fusion_job_sensor_data',
    Base.metadata,
    Column('fusion_job_id', UUID(as_uuid=True), ForeignKey('fusion_jobs.id')),
    Column('sensor_data_id', UUID(as_uuid=True), ForeignKey('sensor_data.id'))
)

class FusionResult(Base):
    """Fusion result database model."""
    __tablename__ = "fusion_results"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    fusion_job_id = Column(UUID(as_uuid=True), ForeignKey("fusion_jobs.id"), nullable=False)
    result_type = Column(String, nullable=False)
    result_data = Column(JSONB, nullable=False)
    confidence = Column(Float, nullable=False)
    metadata = Column(JSONB, default={})
    created_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))

    # Relationships
    fusion_job = relationship("FusionJob")

class AnalyticsSnapshot(Base):
    """Analytics snapshot database model."""
    __tablename__ = "analytics_snapshots"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    snapshot_type = Column(String, nullable=False)
    timestamp = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))
    time_period = Column(String, nullable=False)
    data = Column(JSONB, nullable=False)
    metadata = Column(JSONB, default={})
    created_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))

class AuditLog(Base):
    """Audit log database model."""
    __tablename__ = "audit_logs"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    timestamp = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))
    user_id = Column(UUID(as_uuid=True))
    action = Column(String, nullable=False)
    resource_type = Column(String, nullable=False)
    resource_id = Column(UUID(as_uuid=True))
    details = Column(JSONB, default={})
    ip_address = Column(String)
    user_agent = Column(String)
    created_at = Column(DateTime, nullable=False, default=lambda: datetime.now(timezone.utc))
