"""
Database models for the EW service.
"""

import uuid
from datetime import datetime
from typing import Dict, List, Optional, Any
from sqlalchemy import Column, String, Float, Integer, Boolean, DateTime, ForeignKey, JSON, Text, ARRAY
from sqlalchemy.orm import relationship
from sqlalchemy.dialects.postgresql import UUID, JSONB

from db.session import Base

class EwPlatform(Base):
    """EW platform model."""
    __tablename__ = "ew_platforms"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    type = Column(String, nullable=False)  # drone, vehicle, fixed, etc.
    capabilities = Column(ARRAY(String), nullable=False)  # EA, EP, ES
    frequency_range = Column(JSONB, nullable=False)  # {min_freq, max_freq} in Hz
    power_output = Column(Float, nullable=False)  # Watts
    status = Column(String, nullable=False, default="offline")  # online, offline, error
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    orientation = Column(JSONB, nullable=True)  # {pitch, roll, yaw}
    configuration = Column(JSONB, nullable=True)  # Platform-specific configuration
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    attacks = relationship("ElectronicAttack", back_populates="platform", cascade="all, delete-orphan")
    protections = relationship("ElectronicProtection", back_populates="platform", cascade="all, delete-orphan")
    supports = relationship("ElectronicSupport", back_populates="platform", cascade="all, delete-orphan")
    spectrum_scans = relationship("SpectrumScan", back_populates="platform", cascade="all, delete-orphan")
    countermeasure_deployments = relationship("CountermeasureDeployment", back_populates="platform", cascade="all, delete-orphan")

class ElectronicAttack(Base):
    """Electronic attack model."""
    __tablename__ = "electronic_attacks"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    platform_id = Column(UUID(as_uuid=True), ForeignKey("ew_platforms.id"), nullable=False)
    target_id = Column(UUID(as_uuid=True), nullable=True)
    attack_type = Column(String, nullable=False)  # jamming, spoofing, etc.
    frequency = Column(Float, nullable=False)  # Hz
    bandwidth = Column(Float, nullable=False)  # Hz
    power = Column(Float, nullable=False)  # Watts
    waveform_id = Column(UUID(as_uuid=True), ForeignKey("waveform_templates.id"), nullable=True)
    start_time = Column(DateTime, nullable=False)
    end_time = Column(DateTime, nullable=True)
    status = Column(String, nullable=False)  # scheduled, active, completed, failed
    effectiveness = Column(Float, nullable=True)  # 0.0 to 1.0
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    direction = Column(JSONB, nullable=True)  # {azimuth, elevation}
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    platform = relationship("EwPlatform", back_populates="attacks")
    waveform = relationship("WaveformTemplate", back_populates="attacks")

class ElectronicProtection(Base):
    """Electronic protection model."""
    __tablename__ = "electronic_protections"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    platform_id = Column(UUID(as_uuid=True), ForeignKey("ew_platforms.id"), nullable=False)
    protection_type = Column(String, nullable=False)  # frequency_hopping, directional_filtering, etc.
    frequency_range = Column(JSONB, nullable=False)  # {min_freq, max_freq} in Hz
    start_time = Column(DateTime, nullable=False)
    end_time = Column(DateTime, nullable=True)
    status = Column(String, nullable=False)  # active, inactive
    effectiveness = Column(Float, nullable=True)  # 0.0 to 1.0
    configuration = Column(JSONB, nullable=False)  # Protection-specific configuration
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    platform = relationship("EwPlatform", back_populates="protections")

class ElectronicSupport(Base):
    """Electronic support model."""
    __tablename__ = "electronic_supports"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    platform_id = Column(UUID(as_uuid=True), ForeignKey("ew_platforms.id"), nullable=False)
    support_type = Column(String, nullable=False)  # signal_collection, direction_finding, etc.
    frequency_range = Column(JSONB, nullable=False)  # {min_freq, max_freq} in Hz
    start_time = Column(DateTime, nullable=False)
    end_time = Column(DateTime, nullable=True)
    status = Column(String, nullable=False)  # active, inactive
    configuration = Column(JSONB, nullable=False)  # Support-specific configuration
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    platform = relationship("EwPlatform", back_populates="supports")

class SpectrumScan(Base):
    """Spectrum scan model."""
    __tablename__ = "spectrum_scans"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    platform_id = Column(UUID(as_uuid=True), ForeignKey("ew_platforms.id"), nullable=False)
    start_frequency = Column(Float, nullable=False)  # Hz
    end_frequency = Column(Float, nullable=False)  # Hz
    resolution = Column(Float, nullable=False)  # Hz
    scan_time = Column(DateTime, nullable=False)
    data_url = Column(String, nullable=False)  # URL to spectrum data
    peaks = Column(JSONB, nullable=True)  # List of detected signal peaks
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    
    # Relationships
    platform = relationship("EwPlatform", back_populates="spectrum_scans")

class ElectronicThreat(Base):
    """Electronic threat model."""
    __tablename__ = "electronic_threats"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    threat_type = Column(String, nullable=False)  # jamming, surveillance, etc.
    frequency_range = Column(JSONB, nullable=False)  # {min_freq, max_freq} in Hz
    signal_strength = Column(Float, nullable=False)  # dBm
    first_detected = Column(DateTime, nullable=False)
    last_detected = Column(DateTime, nullable=False)
    location = Column(JSONB, nullable=True)  # {lat, lon, alt}
    direction = Column(JSONB, nullable=True)  # {azimuth, elevation}
    confidence = Column(Float, nullable=False)  # 0.0 to 1.0
    severity = Column(String, nullable=False)  # low, medium, high, critical
    status = Column(String, nullable=False)  # active, inactive
    source_id = Column(UUID(as_uuid=True), nullable=True)  # SIGINT source ID
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    countermeasure_deployments = relationship("CountermeasureDeployment", back_populates="threat")

class Countermeasure(Base):
    """Countermeasure model."""
    __tablename__ = "countermeasures"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    countermeasure_type = Column(String, nullable=False)  # jamming, spoofing, etc.
    threat_types = Column(ARRAY(String), nullable=False)  # Threat types this countermeasure is effective against
    frequency_range = Column(JSONB, nullable=False)  # {min_freq, max_freq} in Hz
    power_required = Column(Float, nullable=False)  # Watts
    effectiveness = Column(Float, nullable=False)  # 0.0 to 1.0
    description = Column(Text, nullable=False)
    configuration_template = Column(JSONB, nullable=False)  # Countermeasure configuration template
    waveform_id = Column(UUID(as_uuid=True), ForeignKey("waveform_templates.id"), nullable=True)
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    waveform = relationship("WaveformTemplate", back_populates="countermeasures")
    deployments = relationship("CountermeasureDeployment", back_populates="countermeasure")

class CountermeasureDeployment(Base):
    """Countermeasure deployment model."""
    __tablename__ = "countermeasure_deployments"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    countermeasure_id = Column(UUID(as_uuid=True), ForeignKey("countermeasures.id"), nullable=False)
    platform_id = Column(UUID(as_uuid=True), ForeignKey("ew_platforms.id"), nullable=False)
    threat_id = Column(UUID(as_uuid=True), ForeignKey("electronic_threats.id"), nullable=True)
    start_time = Column(DateTime, nullable=False)
    end_time = Column(DateTime, nullable=True)
    status = Column(String, nullable=False)  # scheduled, active, completed, failed
    effectiveness = Column(Float, nullable=True)  # 0.0 to 1.0
    configuration = Column(JSONB, nullable=False)  # Deployment-specific configuration
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    countermeasure = relationship("Countermeasure", back_populates="deployments")
    platform = relationship("EwPlatform", back_populates="countermeasure_deployments")
    threat = relationship("ElectronicThreat", back_populates="countermeasure_deployments")

class WaveformTemplate(Base):
    """Waveform template model."""
    __tablename__ = "waveform_templates"
    
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    description = Column(Text, nullable=False)
    waveform_type = Column(String, nullable=False)  # noise, tone, chirp, etc.
    parameters = Column(JSONB, nullable=False)  # Waveform parameters
    file_url = Column(String, nullable=True)  # URL to waveform file
    preview_url = Column(String, nullable=True)  # URL to waveform preview
    metadata = Column(JSONB, nullable=True)  # Additional metadata
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    # Relationships
    attacks = relationship("ElectronicAttack", back_populates="waveform")
    countermeasures = relationship("Countermeasure", back_populates="waveform")
