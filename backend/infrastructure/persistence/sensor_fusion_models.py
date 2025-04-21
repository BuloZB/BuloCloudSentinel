"""
Database models for the Sensor Fusion module.
"""

from sqlalchemy import Column, Integer, String, Float, Boolean, ForeignKey, DateTime, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from datetime import datetime

Base = declarative_base()

class SensorModel(Base):
    """
    Model for storing sensor metadata.
    """
    __tablename__ = "sensors"

    id = Column(Integer, primary_key=True, index=True)
    sensor_id = Column(String(100), unique=True, index=True, nullable=False)
    sensor_type = Column(String(50), nullable=False)
    capabilities = Column(JSON, nullable=True)
    status = Column(String(20), default="registered")
    last_update = Column(DateTime, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    data_points = relationship("SensorDataModel", back_populates="sensor", cascade="all, delete-orphan")


class SensorDataModel(Base):
    """
    Model for storing sensor data points.
    """
    __tablename__ = "sensor_data"

    id = Column(Integer, primary_key=True, index=True)
    sensor_id = Column(Integer, ForeignKey("sensors.id"), nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow, index=True)
    data = Column(JSON, nullable=False)
    
    # Relationships
    sensor = relationship("SensorModel", back_populates="data_points")


class FusedDataModel(Base):
    """
    Model for storing fused data snapshots.
    """
    __tablename__ = "fused_data"

    id = Column(Integer, primary_key=True, index=True)
    timestamp = Column(DateTime, default=datetime.utcnow, index=True)
    position = Column(JSON, nullable=True)
    detections = Column(JSON, nullable=True)
    classifications = Column(JSON, nullable=True)
    telemetry = Column(JSON, nullable=True)


class SensorFusionConfigModel(Base):
    """
    Model for storing sensor fusion configuration.
    """
    __tablename__ = "sensor_fusion_config"

    id = Column(Integer, primary_key=True, index=True)
    key = Column(String(100), unique=True, nullable=False)
    value = Column(JSON, nullable=False)
    description = Column(String(255), nullable=True)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
