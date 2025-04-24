"""
API schemas for the TACS module.
"""

from datetime import datetime
from typing import Dict, List, Optional, Any, Union
from pydantic import BaseModel, Field, UUID4, validator, root_validator
from enum import Enum

# Enums
class TargetType(str, Enum):
    """Target type enumeration."""
    VEHICLE = "vehicle"
    PERSON = "person"
    BUILDING = "building"
    AIRCRAFT = "aircraft"
    VESSEL = "vessel"
    INFRASTRUCTURE = "infrastructure"
    UNKNOWN = "unknown"
    OTHER = "other"

class TargetStatus(str, Enum):
    """Target status enumeration."""
    ACTIVE = "active"
    LOST = "lost"
    ARCHIVED = "archived"

class TrackStatus(str, Enum):
    """Track status enumeration."""
    ACTIVE = "active"
    PAUSED = "paused"
    COMPLETED = "completed"

class SensorType(str, Enum):
    """Sensor type enumeration."""
    EO = "eo"  # Electro-Optical
    IR = "ir"  # Infrared
    RADAR = "radar"
    LIDAR = "lidar"
    MULTISPECTRAL = "multispectral"
    HYPERSPECTRAL = "hyperspectral"
    ACOUSTIC = "acoustic"
    RF = "rf"  # Radio Frequency
    OTHER = "other"

class SensorCapability(str, Enum):
    """Sensor capability enumeration."""
    DETECTION = "detection"
    TRACKING = "tracking"
    CLASSIFICATION = "classification"
    IDENTIFICATION = "identification"
    RANGING = "ranging"
    IMAGING = "imaging"
    MAPPING = "mapping"

class SensorStatus(str, Enum):
    """Sensor status enumeration."""
    ONLINE = "online"
    OFFLINE = "offline"
    DEGRADED = "degraded"
    CALIBRATING = "calibrating"
    STANDBY = "standby"

class PlanStatus(str, Enum):
    """Coordination plan status enumeration."""
    DRAFT = "draft"
    APPROVED = "approved"
    EXECUTING = "executing"
    COMPLETED = "completed"
    ABORTED = "aborted"

class FusionJobStatus(str, Enum):
    """Fusion job status enumeration."""
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"

# Base schemas
class GeoLocation(BaseModel):
    """Geographic location schema."""
    latitude: float = Field(..., ge=-90.0, le=90.0)
    longitude: float = Field(..., ge=-180.0, le=180.0)
    altitude: Optional[float] = None
    accuracy: Optional[float] = None

    @validator('latitude')
    def validate_latitude(cls, v):
        if v < -90.0 or v > 90.0:
            raise ValueError('Latitude must be between -90 and 90 degrees')
        return v

    @validator('longitude')
    def validate_longitude(cls, v):
        if v < -180.0 or v > 180.0:
            raise ValueError('Longitude must be between -180 and 180 degrees')
        return v

class Velocity(BaseModel):
    """Velocity schema."""
    x: float  # m/s in east direction
    y: float  # m/s in north direction
    z: Optional[float] = None  # m/s in up direction
    speed: Optional[float] = None  # m/s total speed
    heading: Optional[float] = None  # degrees from north

    @root_validator
    def calculate_speed_heading(cls, values):
        x = values.get('x')
        y = values.get('y')
        z = values.get('z', 0.0)
        
        if x is not None and y is not None:
            import math
            
            # Calculate speed if not provided
            if values.get('speed') is None:
                values['speed'] = math.sqrt(x**2 + y**2 + z**2)
            
            # Calculate heading if not provided
            if values.get('heading') is None:
                values['heading'] = (math.degrees(math.atan2(x, y)) + 360) % 360
        
        return values

class Acceleration(BaseModel):
    """Acceleration schema."""
    x: float  # m/s^2 in east direction
    y: float  # m/s^2 in north direction
    z: Optional[float] = None  # m/s^2 in up direction

class Dimensions(BaseModel):
    """Physical dimensions schema."""
    length: Optional[float] = None  # meters
    width: Optional[float] = None  # meters
    height: Optional[float] = None  # meters

class Resolution(BaseModel):
    """Sensor resolution schema."""
    horizontal: int  # pixels
    vertical: int  # pixels
    depth: Optional[int] = None  # bits

class FieldOfView(BaseModel):
    """Sensor field of view schema."""
    horizontal: float  # degrees
    vertical: float  # degrees

class SensorConfiguration(BaseModel):
    """Sensor configuration schema."""
    mode: str
    parameters: Dict[str, Any]

class Waypoint(BaseModel):
    """Waypoint schema."""
    location: GeoLocation
    altitude: float
    speed: Optional[float] = None
    heading: Optional[float] = None
    loiter_time: Optional[float] = None
    arrival_time: Optional[datetime] = None
    action: Optional[str] = None
    sensor_actions: Optional[Dict[str, Any]] = None

# Target schemas
class TargetBase(BaseModel):
    """Base target schema."""
    name: str
    type: TargetType
    classification: Optional[str] = None
    confidence: float = Field(..., ge=0.0, le=1.0)
    location: GeoLocation
    velocity: Optional[Velocity] = None
    dimensions: Optional[Dimensions] = None
    source_sensors: List[UUID4] = []
    metadata: Dict[str, Any] = {}
    priority: int = Field(1, ge=1, le=10)
    status: TargetStatus = TargetStatus.ACTIVE

class TargetCreate(TargetBase):
    """Target creation schema."""
    pass

class TargetUpdate(BaseModel):
    """Target update schema."""
    name: Optional[str] = None
    type: Optional[TargetType] = None
    classification: Optional[str] = None
    confidence: Optional[float] = Field(None, ge=0.0, le=1.0)
    location: Optional[GeoLocation] = None
    velocity: Optional[Velocity] = None
    dimensions: Optional[Dimensions] = None
    source_sensors: Optional[List[UUID4]] = None
    metadata: Optional[Dict[str, Any]] = None
    priority: Optional[int] = Field(None, ge=1, le=10)
    status: Optional[TargetStatus] = None

class TargetInDB(TargetBase):
    """Target database schema."""
    id: UUID4
    first_detected: datetime
    last_updated: datetime
    created_at: datetime
    updated_at: datetime

    class Config:
        orm_mode = True

class Target(TargetInDB):
    """Target schema."""
    pass

# Track schemas
class TrackPointBase(BaseModel):
    """Base track point schema."""
    timestamp: datetime
    location: GeoLocation
    altitude: Optional[float] = None
    velocity: Optional[Velocity] = None
    acceleration: Optional[Acceleration] = None
    heading: Optional[float] = None
    confidence: float = Field(..., ge=0.0, le=1.0)
    sensor_id: UUID4

class TrackPoint(TrackPointBase):
    """Track point schema."""
    pass

class TrackBase(BaseModel):
    """Base track schema."""
    target_id: UUID4
    start_time: datetime
    end_time: Optional[datetime] = None
    points: List[TrackPoint] = []
    quality: float = Field(..., ge=0.0, le=1.0)
    status: TrackStatus = TrackStatus.ACTIVE
    metadata: Dict[str, Any] = {}

class TrackCreate(BaseModel):
    """Track creation schema."""
    target_id: UUID4
    start_time: datetime
    points: List[TrackPointBase] = []
    quality: float = Field(..., ge=0.0, le=1.0)
    metadata: Dict[str, Any] = {}

class TrackUpdate(BaseModel):
    """Track update schema."""
    end_time: Optional[datetime] = None
    points: Optional[List[TrackPointBase]] = None
    quality: Optional[float] = Field(None, ge=0.0, le=1.0)
    status: Optional[TrackStatus] = None
    metadata: Optional[Dict[str, Any]] = None

class TrackInDB(TrackBase):
    """Track database schema."""
    id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        orm_mode = True

class Track(TrackInDB):
    """Track schema."""
    pass

# Sensor schemas
class SensorBase(BaseModel):
    """Base sensor schema."""
    name: str
    type: SensorType
    platform_id: UUID4
    capabilities: List[SensorCapability] = []
    resolution: Optional[Resolution] = None
    field_of_view: Optional[FieldOfView] = None
    range: Optional[float] = None
    accuracy: Optional[float] = None
    status: SensorStatus = SensorStatus.ONLINE
    metadata: Dict[str, Any] = {}

class SensorCreate(SensorBase):
    """Sensor creation schema."""
    pass

class SensorUpdate(BaseModel):
    """Sensor update schema."""
    name: Optional[str] = None
    type: Optional[SensorType] = None
    platform_id: Optional[UUID4] = None
    capabilities: Optional[List[SensorCapability]] = None
    resolution: Optional[Resolution] = None
    field_of_view: Optional[FieldOfView] = None
    range: Optional[float] = None
    accuracy: Optional[float] = None
    status: Optional[SensorStatus] = None
    metadata: Optional[Dict[str, Any]] = None

class SensorInDB(SensorBase):
    """Sensor database schema."""
    id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        orm_mode = True

class Sensor(SensorInDB):
    """Sensor schema."""
    pass

# Coordination plan schemas
class CoordinationPlanBase(BaseModel):
    """Base coordination plan schema."""
    name: str
    target_ids: List[UUID4] = []
    platform_ids: List[UUID4] = []
    start_time: datetime
    end_time: Optional[datetime] = None
    waypoints: Dict[str, List[Waypoint]] = {}  # Platform ID -> Waypoints
    sensor_configurations: Dict[str, SensorConfiguration] = {}  # Sensor ID -> Configuration
    priority: int = Field(1, ge=1, le=10)
    status: PlanStatus = PlanStatus.DRAFT
    metadata: Dict[str, Any] = {}

class CoordinationPlanCreate(CoordinationPlanBase):
    """Coordination plan creation schema."""
    pass

class CoordinationPlanUpdate(BaseModel):
    """Coordination plan update schema."""
    name: Optional[str] = None
    target_ids: Optional[List[UUID4]] = None
    platform_ids: Optional[List[UUID4]] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    waypoints: Optional[Dict[str, List[Waypoint]]] = None
    sensor_configurations: Optional[Dict[str, SensorConfiguration]] = None
    priority: Optional[int] = Field(None, ge=1, le=10)
    status: Optional[PlanStatus] = None
    metadata: Optional[Dict[str, Any]] = None

class CoordinationPlanInDB(CoordinationPlanBase):
    """Coordination plan database schema."""
    id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        orm_mode = True

class CoordinationPlan(CoordinationPlanInDB):
    """Coordination plan schema."""
    pass

# Fusion job schemas
class FusionJobBase(BaseModel):
    """Base fusion job schema."""
    sensor_data_ids: List[UUID4]
    target_id: Optional[UUID4] = None
    parameters: Dict[str, Any] = {}
    priority: int = Field(1, ge=1, le=10)
    metadata: Dict[str, Any] = {}

class FusionJobCreate(FusionJobBase):
    """Fusion job creation schema."""
    pass

class FusionJobInDB(FusionJobBase):
    """Fusion job database schema."""
    id: UUID4
    status: FusionJobStatus
    result_id: Optional[UUID4] = None
    error_message: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    completed_at: Optional[datetime] = None

    class Config:
        orm_mode = True

class FusionJob(FusionJobInDB):
    """Fusion job schema."""
    pass

# Analytics schemas
class TargetAnalytics(BaseModel):
    """Target analytics schema."""
    total_targets: int
    targets_by_type: Dict[str, int]
    targets_by_status: Dict[str, int]
    average_confidence: float
    average_track_quality: float
    detection_rate: float  # targets per hour
    time_period: str  # e.g., "last_24h", "last_7d"

class SensorAnalytics(BaseModel):
    """Sensor analytics schema."""
    total_sensors: int
    sensors_by_type: Dict[str, int]
    sensors_by_status: Dict[str, int]
    detection_counts: Dict[str, int]  # sensor_id -> count
    average_confidence: Dict[str, float]  # sensor_id -> confidence
    time_period: str  # e.g., "last_24h", "last_7d"
