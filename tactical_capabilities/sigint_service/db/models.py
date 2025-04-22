"""
Database models for the SIGINT service.
"""

import uuid
from datetime import datetime
from typing import Dict, List, Optional, Any
from sqlalchemy import Column, String, Float, Integer, Boolean, DateTime, ForeignKey, JSON, Text
from sqlalchemy.orm import relationship
from sqlalchemy.dialects.postgresql import UUID, JSONB

from db.session import Base

class SignalCollector(Base):
    """Signal collector model."""
    __tablename__ = "signal_collectors"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    type = Column(String, nullable=False)  # SDR, scanner, spectrum analyzer, etc.
    platform_id = Column(UUID(as_uuid=True), nullable=True)
    status = Column(String, nullable=False, default="offline")  # online, offline, error
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    orientation = Column(JSONB, nullable=True)  # {pitch, roll, yaw}
    frequency_range = Column(JSONB, nullable=False)  # {min_freq, max_freq} in Hz
    capabilities = Column(JSONB, nullable=True)  # Collector-specific capabilities
    configuration = Column(JSONB, nullable=True)  # Collector-specific configuration
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    detections = relationship("SignalDetection", back_populates="collector", cascade="all, delete-orphan")

class SignalDetection(Base):
    """Signal detection model."""
    __tablename__ = "signal_detections"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    collector_id = Column(UUID(as_uuid=True), ForeignKey("signal_collectors.id"), nullable=False)
    source_id = Column(UUID(as_uuid=True), ForeignKey("signal_sources.id"), nullable=True)
    timestamp = Column(DateTime, nullable=False)
    frequency = Column(Float, nullable=False)  # Hz
    bandwidth = Column(Float, nullable=False)  # Hz
    signal_type = Column(String, nullable=True)  # AM, FM, GSM, etc.
    signal_strength = Column(Float, nullable=False)  # dBm
    snr = Column(Float, nullable=False)  # dB
    duration = Column(Float, nullable=False)  # seconds
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    direction = Column(JSONB, nullable=True)  # {azimuth, elevation}
    recording_id = Column(UUID(as_uuid=True), ForeignKey("signal_recordings.id"), nullable=True)
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    collector = relationship("SignalCollector", back_populates="detections")
    source = relationship("SignalSource", back_populates="detections")
    recording = relationship("SignalRecording", back_populates="detections")
    analyses = relationship("SignalAnalysis", back_populates="detection", cascade="all, delete-orphan")

class SignalSource(Base):
    """Signal source model."""
    __tablename__ = "signal_sources"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    first_detected = Column(DateTime, nullable=False)
    last_detected = Column(DateTime, nullable=False)
    status = Column(String, nullable=False, default="active")  # active, inactive, unknown
    signal_type = Column(String, nullable=True)  # AM, FM, GSM, etc.
    frequency_range = Column(JSONB, nullable=False)  # {min_freq, max_freq} in Hz
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    location_accuracy = Column(Float, nullable=True)  # meters
    identification = Column(String, nullable=True)
    threat_level = Column(String, nullable=False, default="none")  # none, low, medium, high, critical
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    detections = relationship("SignalDetection", back_populates="source")

class SignalRecording(Base):
    """Signal recording model."""
    __tablename__ = "signal_recordings"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    start_time = Column(DateTime, nullable=False)
    duration = Column(Float, nullable=False)  # seconds
    sample_rate = Column(Integer, nullable=False)  # Hz
    center_frequency = Column(Float, nullable=False)  # Hz
    bandwidth = Column(Float, nullable=False)  # Hz
    format = Column(String, nullable=False)  # IQ, WAV, etc.
    file_url = Column(String, nullable=False)
    file_size = Column(Integer, nullable=False)  # bytes
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    detections = relationship("SignalDetection", back_populates="recording")

class SignalAnalysis(Base):
    """Signal analysis model."""
    __tablename__ = "signal_analyses"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    detection_id = Column(UUID(as_uuid=True), ForeignKey("signal_detections.id"), nullable=False)
    timestamp = Column(DateTime, nullable=False)
    analysis_type = Column(String, nullable=False)  # classification, decoding, metadata, direction
    confidence = Column(Float, nullable=False)
    results = Column(JSONB, nullable=False)
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    detection = relationship("SignalDetection", back_populates="analyses")

class SigintAlert(Base):
    """SIGINT alert model."""
    __tablename__ = "sigint_alerts"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    type = Column(String, nullable=False)  # new_signal, known_threat, jamming, etc.
    severity = Column(String, nullable=False)  # info, warning, critical
    message = Column(Text, nullable=False)
    timestamp = Column(DateTime, nullable=False)
    frequency = Column(Float, nullable=True)  # Hz
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    source_id = Column(UUID(as_uuid=True), ForeignKey("signal_sources.id"), nullable=True)
    detection_id = Column(UUID(as_uuid=True), ForeignKey("signal_detections.id"), nullable=True)
    acknowledged = Column(Boolean, nullable=False, default=False)
    acknowledged_by = Column(String, nullable=True)
    acknowledged_at = Column(DateTime, nullable=True)
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    source = relationship("SignalSource")
    detection = relationship("SignalDetection")

class SigintIntelligenceProduct(Base):
    """SIGINT intelligence product model."""
    __tablename__ = "sigint_intelligence_products"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String, nullable=False)
    description = Column(Text, nullable=False)
    timestamp = Column(DateTime, nullable=False)
    classification = Column(String, nullable=False, default="unclassified")  # unclassified, confidential, secret, etc.
    sources = Column(JSONB, nullable=False)  # List of source IDs
    detections = Column(JSONB, nullable=False)  # List of detection IDs
    analysis = Column(JSONB, nullable=False)
    conclusions = Column(JSONB, nullable=False)  # List of conclusions
    recommendations = Column(JSONB, nullable=False)  # List of recommendations
    created_by = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)

class KnownSignalProfile(Base):
    """Known signal profile model."""
    __tablename__ = "known_signal_profiles"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    description = Column(Text, nullable=True)
    signal_type = Column(String, nullable=False)  # AM, FM, GSM, etc.
    frequency_range = Column(JSONB, nullable=False)  # {min_freq, max_freq} in Hz
    bandwidth = Column(Float, nullable=False)  # Hz
    modulation = Column(String, nullable=True)
    features = Column(JSONB, nullable=False)  # Signal features for matching
    threat_level = Column(String, nullable=False, default="none")  # none, low, medium, high, critical
    classification = Column(String, nullable=False, default="unclassified")  # unclassified, confidential, secret, etc.
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
