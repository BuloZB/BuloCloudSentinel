"""
API schemas for the EW service.
"""

from datetime import datetime
from typing import Dict, List, Optional, Any, Union
from pydantic import BaseModel, Field, UUID4
from enum import Enum

# Enums
class PlatformType(str, Enum):
    """Platform type enumeration."""
    DRONE = "drone"
    VEHICLE = "vehicle"
    FIXED = "fixed"
    PORTABLE = "portable"
    AIRBORNE = "airborne"
    NAVAL = "naval"
    OTHER = "other"

class EwCapability(str, Enum):
    """EW capability enumeration."""
    EA = "EA"  # Electronic Attack
    EP = "EP"  # Electronic Protection
    ES = "ES"  # Electronic Support

class AttackType(str, Enum):
    """Attack type enumeration."""
    JAMMING = "jamming"
    SPOOFING = "spoofing"
    DECEPTION = "deception"
    MEACONING = "meaconing"
    DIRECTED_ENERGY = "directed_energy"
    OTHER = "other"

class ProtectionType(str, Enum):
    """Protection type enumeration."""
    FREQUENCY_HOPPING = "frequency_hopping"
    DIRECTIONAL_FILTERING = "directional_filtering"
    SPREAD_SPECTRUM = "spread_spectrum"
    ENCRYPTION = "encryption"
    ANTI_JAM = "anti_jam"
    OTHER = "other"

class SupportType(str, Enum):
    """Support type enumeration."""
    SIGNAL_COLLECTION = "signal_collection"
    DIRECTION_FINDING = "direction_finding"
    SIGNAL_ANALYSIS = "signal_analysis"
    THREAT_DETECTION = "threat_detection"
    SPECTRUM_MONITORING = "spectrum_monitoring"
    OTHER = "other"

class ThreatType(str, Enum):
    """Threat type enumeration."""
    JAMMING = "jamming"
    SURVEILLANCE = "surveillance"
    SPOOFING = "spoofing"
    DECEPTION = "deception"
    MEACONING = "meaconing"
    DIRECTED_ENERGY = "directed_energy"
    OTHER = "other"

class ThreatSeverity(str, Enum):
    """Threat severity enumeration."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"

class ThreatStatus(str, Enum):
    """Threat status enumeration."""
    ACTIVE = "active"
    INACTIVE = "inactive"

class CountermeasureType(str, Enum):
    """Countermeasure type enumeration."""
    JAMMING = "jamming"
    SPOOFING = "spoofing"
    DECEPTION = "deception"
    EVASION = "evasion"
    HARDENING = "hardening"
    OTHER = "other"

class WaveformType(str, Enum):
    """Waveform type enumeration."""
    NOISE = "noise"
    TONE = "tone"
    CHIRP = "chirp"
    SWEEP = "sweep"
    PULSE = "pulse"
    CUSTOM = "custom"
    OTHER = "other"

class DeploymentStatus(str, Enum):
    """Deployment status enumeration."""
    SCHEDULED = "scheduled"
    ACTIVE = "active"
    COMPLETED = "completed"
    FAILED = "failed"

# Base schemas
class LocationSchema(BaseModel):
    """Location schema."""
    lat: float
    lon: float
    alt: Optional[float] = None

class OrientationSchema(BaseModel):
    """Orientation schema."""
    pitch: Optional[float] = None
    roll: Optional[float] = None
    yaw: Optional[float] = None

class DirectionSchema(BaseModel):
    """Direction schema."""
    azimuth: float  # degrees from north
    elevation: Optional[float] = None  # degrees from horizontal

class FrequencyRangeSchema(BaseModel):
    """Frequency range schema."""
    min_freq: float  # Hz
    max_freq: float  # Hz

# EW platform schemas
class EwPlatformBase(BaseModel):
    """Base EW platform schema."""
    name: str
    type: PlatformType
    capabilities: List[EwCapability]
    frequency_range: FrequencyRangeSchema
    power_output: float  # Watts
    location: Optional[LocationSchema] = None
    orientation: Optional[OrientationSchema] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class EwPlatformCreate(EwPlatformBase):
    """EW platform creation schema."""
    pass

class EwPlatformUpdate(BaseModel):
    """EW platform update schema."""
    name: Optional[str] = None
    type: Optional[PlatformType] = None
    capabilities: Optional[List[EwCapability]] = None
    frequency_range: Optional[FrequencyRangeSchema] = None
    power_output: Optional[float] = None
    status: Optional[str] = None
    location: Optional[LocationSchema] = None
    orientation: Optional[OrientationSchema] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class EwPlatformInDB(EwPlatformBase):
    """EW platform database schema."""
    id: UUID4
    status: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class EwPlatform(EwPlatformInDB):
    """EW platform schema."""
    pass

# Electronic attack schemas
class ElectronicAttackBase(BaseModel):
    """Base electronic attack schema."""
    platform_id: UUID4
    target_id: Optional[UUID4] = None
    attack_type: AttackType
    frequency: float  # Hz
    bandwidth: float  # Hz
    power: float  # Watts
    waveform_id: Optional[UUID4] = None
    start_time: datetime
    end_time: Optional[datetime] = None
    location: Optional[LocationSchema] = None
    direction: Optional[DirectionSchema] = None
    metadata: Optional[Dict[str, Any]] = None

class ElectronicAttackCreate(ElectronicAttackBase):
    """Electronic attack creation schema."""
    pass

class ElectronicAttackUpdate(BaseModel):
    """Electronic attack update schema."""
    target_id: Optional[UUID4] = None
    attack_type: Optional[AttackType] = None
    frequency: Optional[float] = None
    bandwidth: Optional[float] = None
    power: Optional[float] = None
    waveform_id: Optional[UUID4] = None
    end_time: Optional[datetime] = None
    status: Optional[str] = None
    effectiveness: Optional[float] = None
    location: Optional[LocationSchema] = None
    direction: Optional[DirectionSchema] = None
    metadata: Optional[Dict[str, Any]] = None

class ElectronicAttackInDB(ElectronicAttackBase):
    """Electronic attack database schema."""
    id: UUID4
    status: str
    effectiveness: Optional[float] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class ElectronicAttack(ElectronicAttackInDB):
    """Electronic attack schema."""
    pass

# Electronic protection schemas
class ElectronicProtectionBase(BaseModel):
    """Base electronic protection schema."""
    platform_id: UUID4
    protection_type: ProtectionType
    frequency_range: FrequencyRangeSchema
    start_time: datetime
    end_time: Optional[datetime] = None
    configuration: Dict[str, Any]
    metadata: Optional[Dict[str, Any]] = None

class ElectronicProtectionCreate(ElectronicProtectionBase):
    """Electronic protection creation schema."""
    pass

class ElectronicProtectionUpdate(BaseModel):
    """Electronic protection update schema."""
    protection_type: Optional[ProtectionType] = None
    frequency_range: Optional[FrequencyRangeSchema] = None
    end_time: Optional[datetime] = None
    status: Optional[str] = None
    effectiveness: Optional[float] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class ElectronicProtectionInDB(ElectronicProtectionBase):
    """Electronic protection database schema."""
    id: UUID4
    status: str
    effectiveness: Optional[float] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class ElectronicProtection(ElectronicProtectionInDB):
    """Electronic protection schema."""
    pass

# Electronic support schemas
class ElectronicSupportBase(BaseModel):
    """Base electronic support schema."""
    platform_id: UUID4
    support_type: SupportType
    frequency_range: FrequencyRangeSchema
    start_time: datetime
    end_time: Optional[datetime] = None
    configuration: Dict[str, Any]
    metadata: Optional[Dict[str, Any]] = None

class ElectronicSupportCreate(ElectronicSupportBase):
    """Electronic support creation schema."""
    pass

class ElectronicSupportUpdate(BaseModel):
    """Electronic support update schema."""
    support_type: Optional[SupportType] = None
    frequency_range: Optional[FrequencyRangeSchema] = None
    end_time: Optional[datetime] = None
    status: Optional[str] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class ElectronicSupportInDB(ElectronicSupportBase):
    """Electronic support database schema."""
    id: UUID4
    status: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class ElectronicSupport(ElectronicSupportInDB):
    """Electronic support schema."""
    pass

# Spectrum scan schemas
class SpectrumScanBase(BaseModel):
    """Base spectrum scan schema."""
    platform_id: UUID4
    start_frequency: float  # Hz
    end_frequency: float  # Hz
    resolution: float  # Hz
    scan_time: datetime
    data_url: str
    peaks: Optional[List[Dict[str, Any]]] = None
    metadata: Optional[Dict[str, Any]] = None

class SpectrumScanCreate(SpectrumScanBase):
    """Spectrum scan creation schema."""
    pass

class SpectrumScanInDB(SpectrumScanBase):
    """Spectrum scan database schema."""
    id: UUID4
    created_at: datetime

    class Config:
        from_attributes = True

class SpectrumScan(SpectrumScanInDB):
    """Spectrum scan schema."""
    pass

# Electronic threat schemas
class ElectronicThreatBase(BaseModel):
    """Base electronic threat schema."""
    threat_type: ThreatType
    frequency_range: FrequencyRangeSchema
    signal_strength: float  # dBm
    first_detected: datetime
    last_detected: datetime
    location: Optional[LocationSchema] = None
    direction: Optional[DirectionSchema] = None
    confidence: float  # 0.0 to 1.0
    severity: ThreatSeverity
    status: ThreatStatus
    source_id: Optional[UUID4] = None
    metadata: Optional[Dict[str, Any]] = None

class ElectronicThreatCreate(ElectronicThreatBase):
    """Electronic threat creation schema."""
    pass

class ElectronicThreatUpdate(BaseModel):
    """Electronic threat update schema."""
    threat_type: Optional[ThreatType] = None
    frequency_range: Optional[FrequencyRangeSchema] = None
    signal_strength: Optional[float] = None
    last_detected: Optional[datetime] = None
    location: Optional[LocationSchema] = None
    direction: Optional[DirectionSchema] = None
    confidence: Optional[float] = None
    severity: Optional[ThreatSeverity] = None
    status: Optional[ThreatStatus] = None
    source_id: Optional[UUID4] = None
    metadata: Optional[Dict[str, Any]] = None

class ElectronicThreatInDB(ElectronicThreatBase):
    """Electronic threat database schema."""
    id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class ElectronicThreat(ElectronicThreatInDB):
    """Electronic threat schema."""
    pass

# Countermeasure schemas
class CountermeasureBase(BaseModel):
    """Base countermeasure schema."""
    name: str
    countermeasure_type: CountermeasureType
    threat_types: List[ThreatType]
    frequency_range: FrequencyRangeSchema
    power_required: float  # Watts
    effectiveness: float  # 0.0 to 1.0
    description: str
    configuration_template: Dict[str, Any]
    waveform_id: Optional[UUID4] = None
    metadata: Optional[Dict[str, Any]] = None

class CountermeasureCreate(CountermeasureBase):
    """Countermeasure creation schema."""
    pass

class CountermeasureUpdate(BaseModel):
    """Countermeasure update schema."""
    name: Optional[str] = None
    countermeasure_type: Optional[CountermeasureType] = None
    threat_types: Optional[List[ThreatType]] = None
    frequency_range: Optional[FrequencyRangeSchema] = None
    power_required: Optional[float] = None
    effectiveness: Optional[float] = None
    description: Optional[str] = None
    configuration_template: Optional[Dict[str, Any]] = None
    waveform_id: Optional[UUID4] = None
    metadata: Optional[Dict[str, Any]] = None

class CountermeasureInDB(CountermeasureBase):
    """Countermeasure database schema."""
    id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class Countermeasure(CountermeasureInDB):
    """Countermeasure schema."""
    pass

# Countermeasure deployment schemas
class CountermeasureDeploymentBase(BaseModel):
    """Base countermeasure deployment schema."""
    countermeasure_id: UUID4
    platform_id: UUID4
    threat_id: Optional[UUID4] = None
    start_time: datetime
    end_time: Optional[datetime] = None
    configuration: Dict[str, Any]
    metadata: Optional[Dict[str, Any]] = None

class CountermeasureDeploymentCreate(CountermeasureDeploymentBase):
    """Countermeasure deployment creation schema."""
    pass

class CountermeasureDeploymentUpdate(BaseModel):
    """Countermeasure deployment update schema."""
    end_time: Optional[datetime] = None
    status: Optional[DeploymentStatus] = None
    effectiveness: Optional[float] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class CountermeasureDeploymentInDB(CountermeasureDeploymentBase):
    """Countermeasure deployment database schema."""
    id: UUID4
    status: DeploymentStatus
    effectiveness: Optional[float] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class CountermeasureDeployment(CountermeasureDeploymentInDB):
    """Countermeasure deployment schema."""
    pass

# Waveform template schemas
class WaveformTemplateBase(BaseModel):
    """Base waveform template schema."""
    name: str
    description: str
    waveform_type: WaveformType
    parameters: Dict[str, Any]
    file_url: Optional[str] = None
    preview_url: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

class WaveformTemplateCreate(WaveformTemplateBase):
    """Waveform template creation schema."""
    pass

class WaveformTemplateUpdate(BaseModel):
    """Waveform template update schema."""
    name: Optional[str] = None
    description: Optional[str] = None
    waveform_type: Optional[WaveformType] = None
    parameters: Optional[Dict[str, Any]] = None
    file_url: Optional[str] = None
    preview_url: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

class WaveformTemplateInDB(WaveformTemplateBase):
    """Waveform template database schema."""
    id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class WaveformTemplate(WaveformTemplateInDB):
    """Waveform template schema."""
    pass

# Request schemas
class SpectrumScanRequest(BaseModel):
    """Spectrum scan request schema."""
    platform_id: UUID4
    start_frequency: float  # Hz
    end_frequency: float  # Hz
    resolution: Optional[float] = None  # Hz
    metadata: Optional[Dict[str, Any]] = None

class CountermeasureDeployRequest(BaseModel):
    """Countermeasure deployment request schema."""
    countermeasure_id: UUID4
    platform_id: UUID4
    threat_id: Optional[UUID4] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None
