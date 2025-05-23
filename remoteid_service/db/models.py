"""
Database models for the Remote ID & Regulatory Compliance Service.

This module defines SQLAlchemy ORM models for the service's database.
"""

import uuid
from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional, Any

from sqlalchemy import (
    Boolean,
    Column,
    DateTime,
    Enum as SQLAlchemyEnum,
    Float,
    ForeignKey,
    Integer,
    String,
    Table,
    Text,
)
from sqlalchemy.dialects.postgresql import JSONB, UUID
from sqlalchemy.orm import relationship
from geoalchemy2 import Geography, Geometry

from remoteid_service.db.session import Base

# Enums
class RemoteIDMode(str, Enum):
    """Remote ID broadcast mode."""
    FAA = "faa"
    EU = "eu"
    CUSTOM = "custom"

class BroadcastMethod(str, Enum):
    """Remote ID broadcast method."""
    WIFI_NAN = "wifi_nan"
    BLUETOOTH_LE = "bluetooth_le"
    NETWORK = "network"
    ASTM_NETWORK = "astm_network"

class FlightPlanStatus(str, Enum):
    """Flight plan status."""
    DRAFT = "draft"
    SUBMITTED = "submitted"
    APPROVED = "approved"
    REJECTED = "rejected"
    CANCELLED = "cancelled"
    COMPLETED = "completed"

class FlightPlanType(str, Enum):
    """Flight plan type."""
    EASA_SORA = "easa_sora"
    FAA_LAANC = "faa_laanc"
    CUSTOM = "custom"

class NOTAMType(str, Enum):
    """NOTAM type."""
    AIRSPACE = "airspace"
    OBSTACLE = "obstacle"
    AIRPORT = "airport"
    PROCEDURE = "procedure"
    OTHER = "other"

# Models
class RemoteIDBroadcast(Base):
    """Remote ID broadcast record."""
    __tablename__ = "remoteid_broadcasts"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    drone_id = Column(String(50), nullable=False, index=True)
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow, index=True)
    mode = Column(SQLAlchemyEnum(RemoteIDMode), nullable=False)
    method = Column(SQLAlchemyEnum(BroadcastMethod), nullable=False)
    location = Column(Geography(geometry_type="POINT", srid=4326), nullable=False)
    altitude = Column(Float, nullable=False)
    speed = Column(Float, nullable=True)
    heading = Column(Float, nullable=True)
    operator_id = Column(String(100), nullable=True)
    serial_number = Column(String(100), nullable=True)
    session_id = Column(String(100), nullable=True)
    message_data = Column(JSONB, nullable=True)
    
    def __repr__(self) -> str:
        return f"<RemoteIDBroadcast(id={self.id}, drone_id={self.drone_id}, timestamp={self.timestamp})>"

class FlightPlan(Base):
    """Flight plan."""
    __tablename__ = "flight_plans"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String(100), nullable=False)
    description = Column(Text, nullable=True)
    operator_id = Column(String(100), nullable=False, index=True)
    drone_id = Column(String(50), nullable=False, index=True)
    plan_type = Column(SQLAlchemyEnum(FlightPlanType), nullable=False)
    status = Column(SQLAlchemyEnum(FlightPlanStatus), nullable=False, default=FlightPlanStatus.DRAFT)
    start_time = Column(DateTime, nullable=False)
    end_time = Column(DateTime, nullable=False)
    max_altitude = Column(Float, nullable=False)
    area = Column(Geography(geometry_type="POLYGON", srid=4326), nullable=False)
    path = Column(Geography(geometry_type="LINESTRING", srid=4326), nullable=True)
    submission_id = Column(String(100), nullable=True)
    submission_time = Column(DateTime, nullable=True)
    approval_time = Column(DateTime, nullable=True)
    rejection_reason = Column(Text, nullable=True)
    metadata = Column(JSONB, nullable=True)
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    updated_at = Column(DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    waypoints = relationship("Waypoint", back_populates="flight_plan", cascade="all, delete-orphan")
    
    def __repr__(self) -> str:
        return f"<FlightPlan(id={self.id}, name={self.name}, status={self.status})>"

class Waypoint(Base):
    """Waypoint in a flight plan."""
    __tablename__ = "waypoints"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    flight_plan_id = Column(UUID(as_uuid=True), ForeignKey("flight_plans.id"), nullable=False)
    sequence = Column(Integer, nullable=False)
    location = Column(Geography(geometry_type="POINT", srid=4326), nullable=False)
    altitude = Column(Float, nullable=False)
    speed = Column(Float, nullable=True)
    hold_time = Column(Integer, nullable=True)  # seconds
    action = Column(String(50), nullable=True)
    parameters = Column(JSONB, nullable=True)
    
    # Relationships
    flight_plan = relationship("FlightPlan", back_populates="waypoints")
    
    def __repr__(self) -> str:
        return f"<Waypoint(id={self.id}, sequence={self.sequence})>"

class NOTAM(Base):
    """Notice to Air Missions (NOTAM)."""
    __tablename__ = "notams"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    notam_id = Column(String(50), nullable=False, unique=True, index=True)
    source = Column(String(50), nullable=False)
    notam_type = Column(SQLAlchemyEnum(NOTAMType), nullable=False)
    location = Column(String(50), nullable=False, index=True)
    effective_start = Column(DateTime, nullable=False)
    effective_end = Column(DateTime, nullable=False)
    altitude_lower = Column(Float, nullable=True)
    altitude_upper = Column(Float, nullable=True)
    area = Column(Geography(geometry_type="POLYGON", srid=4326), nullable=True)
    point = Column(Geography(geometry_type="POINT", srid=4326), nullable=True)
    description = Column(Text, nullable=False)
    raw_text = Column(Text, nullable=True)
    metadata = Column(JSONB, nullable=True)
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    updated_at = Column(DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    def __repr__(self) -> str:
        return f"<NOTAM(id={self.id}, notam_id={self.notam_id})>"
