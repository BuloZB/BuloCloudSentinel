"""
API schemas for the SIGINT service.
"""

from datetime import datetime
from typing import Dict, List, Optional, Any, Union
from pydantic import BaseModel, Field, UUID4
from enum import Enum

# Enums
class CollectorType(str, Enum):
    """Collector type enumeration."""
    SDR = "SDR"  # Software Defined Radio
    SCANNER = "SCANNER"
    SPECTRUM_ANALYZER = "SPECTRUM_ANALYZER"
    WIDEBAND_RECEIVER = "WIDEBAND_RECEIVER"
    DIRECTION_FINDER = "DIRECTION_FINDER"
    MULTI = "MULTI"  # Multi-collector
    OTHER = "OTHER"

class SignalType(str, Enum):
    """Signal type enumeration."""
    AM = "AM"
    FM = "FM"
    SSB = "SSB"
    CW = "CW"
    FSK = "FSK"
    PSK = "PSK"
    QAM = "QAM"
    OFDM = "OFDM"
    GSM = "GSM"
    LTE = "LTE"
    WIFI = "WIFI"
    BLUETOOTH = "BLUETOOTH"
    UNKNOWN = "UNKNOWN"
    OTHER = "OTHER"

class SourceStatus(str, Enum):
    """Source status enumeration."""
    ACTIVE = "active"
    INACTIVE = "inactive"
    UNKNOWN = "unknown"

class ThreatLevel(str, Enum):
    """Threat level enumeration."""
    NONE = "none"
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"

class AlertType(str, Enum):
    """Alert type enumeration."""
    NEW_SIGNAL = "new_signal"
    KNOWN_THREAT = "known_threat"
    JAMMING = "jamming"
    UNAUTHORIZED = "unauthorized"
    ANOMALY = "anomaly"
    OTHER = "other"

class AlertSeverity(str, Enum):
    """Alert severity enumeration."""
    INFO = "info"
    WARNING = "warning"
    CRITICAL = "critical"

class AnalysisType(str, Enum):
    """Analysis type enumeration."""
    CLASSIFICATION = "classification"
    DECODING = "decoding"
    METADATA = "metadata"
    DIRECTION = "direction"

class Classification(str, Enum):
    """Classification level enumeration."""
    UNCLASSIFIED = "unclassified"
    CONFIDENTIAL = "confidential"
    SECRET = "secret"
    TOP_SECRET = "top_secret"

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

# Signal collector schemas
class SignalCollectorBase(BaseModel):
    """Base signal collector schema."""
    name: str
    type: CollectorType
    platform_id: Optional[UUID4] = None
    location: Optional[LocationSchema] = None
    orientation: Optional[OrientationSchema] = None
    frequency_range: FrequencyRangeSchema
    capabilities: Optional[Dict[str, Any]] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class SignalCollectorCreate(SignalCollectorBase):
    """Signal collector creation schema."""
    pass

class SignalCollectorUpdate(BaseModel):
    """Signal collector update schema."""
    name: Optional[str] = None
    type: Optional[CollectorType] = None
    platform_id: Optional[UUID4] = None
    status: Optional[str] = None
    location: Optional[LocationSchema] = None
    orientation: Optional[OrientationSchema] = None
    frequency_range: Optional[FrequencyRangeSchema] = None
    capabilities: Optional[Dict[str, Any]] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class SignalCollectorInDB(SignalCollectorBase):
    """Signal collector database schema."""
    id: UUID4
    status: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class SignalCollector(SignalCollectorInDB):
    """Signal collector schema."""
    pass

# Signal detection schemas
class SignalDetectionBase(BaseModel):
    """Base signal detection schema."""
    collector_id: UUID4
    timestamp: datetime
    frequency: float  # Hz
    bandwidth: float  # Hz
    signal_type: Optional[SignalType] = None
    signal_strength: float  # dBm
    snr: float  # dB
    duration: float  # seconds
    location: Optional[LocationSchema] = None
    direction: Optional[DirectionSchema] = None
    recording_id: Optional[UUID4] = None
    metadata: Optional[Dict[str, Any]] = None

class SignalDetectionCreate(SignalDetectionBase):
    """Signal detection creation schema."""
    pass

class SignalDetectionInDB(SignalDetectionBase):
    """Signal detection database schema."""
    id: UUID4
    source_id: Optional[UUID4] = None
    created_at: datetime

    class Config:
        from_attributes = True

class SignalDetection(SignalDetectionInDB):
    """Signal detection schema."""
    pass

# Signal source schemas
class SignalSourceBase(BaseModel):
    """Base signal source schema."""
    first_detected: datetime
    last_detected: datetime
    signal_type: Optional[SignalType] = None
    frequency_range: FrequencyRangeSchema
    location: Optional[LocationSchema] = None
    location_accuracy: Optional[float] = None  # meters
    identification: Optional[str] = None
    threat_level: ThreatLevel = ThreatLevel.NONE
    metadata: Optional[Dict[str, Any]] = None

class SignalSourceCreate(SignalSourceBase):
    """Signal source creation schema."""
    pass

class SignalSourceUpdate(BaseModel):
    """Signal source update schema."""
    last_detected: Optional[datetime] = None
    status: Optional[SourceStatus] = None
    signal_type: Optional[SignalType] = None
    frequency_range: Optional[FrequencyRangeSchema] = None
    location: Optional[LocationSchema] = None
    location_accuracy: Optional[float] = None
    identification: Optional[str] = None
    threat_level: Optional[ThreatLevel] = None
    metadata: Optional[Dict[str, Any]] = None

class SignalSourceInDB(SignalSourceBase):
    """Signal source database schema."""
    id: UUID4
    status: SourceStatus
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class SignalSource(SignalSourceInDB):
    """Signal source schema."""
    detections: List[SignalDetection] = []

# Signal recording schemas
class SignalRecordingBase(BaseModel):
    """Base signal recording schema."""
    start_time: datetime
    duration: float  # seconds
    sample_rate: int  # Hz
    center_frequency: float  # Hz
    bandwidth: float  # Hz
    format: str  # IQ, WAV, etc.
    file_url: str
    file_size: int  # bytes
    metadata: Optional[Dict[str, Any]] = None

class SignalRecordingCreate(SignalRecordingBase):
    """Signal recording creation schema."""
    pass

class SignalRecordingInDB(SignalRecordingBase):
    """Signal recording database schema."""
    id: UUID4
    created_at: datetime

    class Config:
        from_attributes = True

class SignalRecording(SignalRecordingInDB):
    """Signal recording schema."""
    pass

# Signal analysis schemas
class SignalAnalysisBase(BaseModel):
    """Base signal analysis schema."""
    detection_id: UUID4
    timestamp: datetime
    analysis_type: AnalysisType
    confidence: float
    results: Dict[str, Any]
    metadata: Optional[Dict[str, Any]] = None

class SignalAnalysisCreate(SignalAnalysisBase):
    """Signal analysis creation schema."""
    pass

class SignalAnalysisInDB(SignalAnalysisBase):
    """Signal analysis database schema."""
    id: UUID4
    created_at: datetime

    class Config:
        from_attributes = True

class SignalAnalysis(SignalAnalysisInDB):
    """Signal analysis schema."""
    pass

# SIGINT alert schemas
class SigintAlertBase(BaseModel):
    """Base SIGINT alert schema."""
    type: AlertType
    severity: AlertSeverity
    message: str
    timestamp: datetime
    frequency: Optional[float] = None  # Hz
    location: Optional[LocationSchema] = None
    source_id: Optional[UUID4] = None
    detection_id: Optional[UUID4] = None
    metadata: Optional[Dict[str, Any]] = None

class SigintAlertCreate(SigintAlertBase):
    """SIGINT alert creation schema."""
    pass

class SigintAlertUpdate(BaseModel):
    """SIGINT alert update schema."""
    acknowledged: bool = True
    acknowledged_by: Optional[str] = None
    acknowledged_at: Optional[datetime] = None

class SigintAlertInDB(SigintAlertBase):
    """SIGINT alert database schema."""
    id: UUID4
    acknowledged: bool = False
    acknowledged_by: Optional[str] = None
    acknowledged_at: Optional[datetime] = None
    created_at: datetime

    class Config:
        from_attributes = True

class SigintAlert(SigintAlertInDB):
    """SIGINT alert schema."""
    pass

# SIGINT intelligence product schemas
class SigintIntelligenceProductBase(BaseModel):
    """Base SIGINT intelligence product schema."""
    title: str
    description: str
    timestamp: datetime
    classification: Classification = Classification.UNCLASSIFIED
    sources: List[UUID4]
    detections: List[UUID4]
    analysis: Dict[str, Any]
    conclusions: List[str]
    recommendations: List[str]

class SigintIntelligenceProductCreate(SigintIntelligenceProductBase):
    """SIGINT intelligence product creation schema."""
    pass

class SigintIntelligenceProductInDB(SigintIntelligenceProductBase):
    """SIGINT intelligence product database schema."""
    id: UUID4
    created_by: str
    created_at: datetime

    class Config:
        from_attributes = True

class SigintIntelligenceProduct(SigintIntelligenceProductInDB):
    """SIGINT intelligence product schema."""
    pass

# Known signal profile schemas
class KnownSignalProfileBase(BaseModel):
    """Base known signal profile schema."""
    name: str
    description: Optional[str] = None
    signal_type: SignalType
    frequency_range: FrequencyRangeSchema
    bandwidth: float  # Hz
    modulation: Optional[str] = None
    features: Dict[str, Any]
    threat_level: ThreatLevel = ThreatLevel.NONE
    classification: Classification = Classification.UNCLASSIFIED
    metadata: Optional[Dict[str, Any]] = None

class KnownSignalProfileCreate(KnownSignalProfileBase):
    """Known signal profile creation schema."""
    pass

class KnownSignalProfileUpdate(BaseModel):
    """Known signal profile update schema."""
    name: Optional[str] = None
    description: Optional[str] = None
    signal_type: Optional[SignalType] = None
    frequency_range: Optional[FrequencyRangeSchema] = None
    bandwidth: Optional[float] = None
    modulation: Optional[str] = None
    features: Optional[Dict[str, Any]] = None
    threat_level: Optional[ThreatLevel] = None
    classification: Optional[Classification] = None
    metadata: Optional[Dict[str, Any]] = None

class KnownSignalProfileInDB(KnownSignalProfileBase):
    """Known signal profile database schema."""
    id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class KnownSignalProfile(KnownSignalProfileInDB):
    """Known signal profile schema."""
    pass

# Analysis request schemas
class ClassifySignalRequest(BaseModel):
    """Classify signal request schema."""
    detection_id: Optional[UUID4] = None
    recording_id: Optional[UUID4] = None
    signal_data: Optional[List[float]] = None
    sample_rate: Optional[int] = None
    center_frequency: Optional[float] = None

class DecodeSignalRequest(BaseModel):
    """Decode signal request schema."""
    detection_id: Optional[UUID4] = None
    recording_id: Optional[UUID4] = None
    signal_type: Optional[SignalType] = None
    signal_data: Optional[List[float]] = None
    sample_rate: Optional[int] = None
    center_frequency: Optional[float] = None

class ExtractMetadataRequest(BaseModel):
    """Extract metadata request schema."""
    detection_id: Optional[UUID4] = None
    recording_id: Optional[UUID4] = None
    signal_type: Optional[SignalType] = None
    signal_data: Optional[List[float]] = None
    sample_rate: Optional[int] = None
    center_frequency: Optional[float] = None

class DirectionFindingRequest(BaseModel):
    """Direction finding request schema."""
    detections: List[UUID4]
    max_age: Optional[int] = None  # seconds
