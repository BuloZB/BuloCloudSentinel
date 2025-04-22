"""
API schemas for the ISR service.
"""

from datetime import datetime
from typing import Dict, List, Optional, Any, Union
from pydantic import BaseModel, Field, UUID4
from enum import Enum

# Enums
class SensorType(str, Enum):
    """Sensor type enumeration."""
    EO = "EO"  # Electro-Optical
    IR = "IR"  # Infrared
    RADAR = "RADAR"
    LIDAR = "LIDAR"
    ACOUSTIC = "ACOUSTIC"
    RF = "RF"  # Radio Frequency
    MULTI = "MULTI"  # Multi-sensor
    OTHER = "OTHER"

class PlatformType(str, Enum):
    """Platform type enumeration."""
    DRONE = "drone"
    GROUND_STATION = "ground_station"
    VEHICLE = "vehicle"
    FIXED = "fixed"
    SATELLITE = "satellite"
    OTHER = "other"

class ObjectType(str, Enum):
    """Object type enumeration."""
    PERSON = "person"
    VEHICLE = "vehicle"
    AIRCRAFT = "aircraft"
    VESSEL = "vessel"
    BUILDING = "building"
    ANIMAL = "animal"
    UNKNOWN = "unknown"
    OTHER = "other"

class TargetStatus(str, Enum):
    """Target status enumeration."""
    ACTIVE = "active"
    LOST = "lost"
    ARCHIVED = "archived"

class AlertType(str, Enum):
    """Alert type enumeration."""
    INTRUSION = "intrusion"
    BEHAVIOR = "behavior"
    SYSTEM = "system"
    TECHNICAL = "technical"
    OTHER = "other"

class AlertSeverity(str, Enum):
    """Alert severity enumeration."""
    INFO = "info"
    WARNING = "warning"
    CRITICAL = "critical"

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

class BoundingBoxSchema(BaseModel):
    """Bounding box schema."""
    x: float
    y: float
    width: float
    height: float

class VelocitySchema(BaseModel):
    """Velocity schema."""
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None

# Sensor schemas
class SensorBase(BaseModel):
    """Base sensor schema."""
    name: str
    type: SensorType
    platform_id: Optional[UUID4] = None
    location: Optional[LocationSchema] = None
    orientation: Optional[OrientationSchema] = None
    capabilities: Optional[Dict[str, Any]] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class SensorCreate(SensorBase):
    """Sensor creation schema."""
    pass

class SensorUpdate(BaseModel):
    """Sensor update schema."""
    name: Optional[str] = None
    type: Optional[SensorType] = None
    platform_id: Optional[UUID4] = None
    status: Optional[str] = None
    location: Optional[LocationSchema] = None
    orientation: Optional[OrientationSchema] = None
    capabilities: Optional[Dict[str, Any]] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class SensorInDB(SensorBase):
    """Sensor database schema."""
    id: UUID4
    status: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class Sensor(SensorInDB):
    """Sensor schema."""
    pass

# Platform schemas
class PlatformBase(BaseModel):
    """Base platform schema."""
    name: str
    type: PlatformType
    location: Optional[LocationSchema] = None
    orientation: Optional[OrientationSchema] = None
    capabilities: Optional[Dict[str, Any]] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class PlatformCreate(PlatformBase):
    """Platform creation schema."""
    pass

class PlatformUpdate(BaseModel):
    """Platform update schema."""
    name: Optional[str] = None
    type: Optional[PlatformType] = None
    status: Optional[str] = None
    location: Optional[LocationSchema] = None
    orientation: Optional[OrientationSchema] = None
    capabilities: Optional[Dict[str, Any]] = None
    configuration: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class PlatformInDB(PlatformBase):
    """Platform database schema."""
    id: UUID4
    status: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class Platform(PlatformInDB):
    """Platform schema."""
    sensors: List[Sensor] = []

# Observation schemas
class ObservationBase(BaseModel):
    """Base observation schema."""
    sensor_id: UUID4
    timestamp: datetime
    data_type: str
    data_url: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    location: Optional[LocationSchema] = None
    orientation: Optional[OrientationSchema] = None

class ObservationCreate(ObservationBase):
    """Observation creation schema."""
    pass

class ObservationInDB(ObservationBase):
    """Observation database schema."""
    id: UUID4
    created_at: datetime

    class Config:
        from_attributes = True

class Observation(ObservationInDB):
    """Observation schema."""
    pass

# Detection schemas
class DetectionBase(BaseModel):
    """Base detection schema."""
    observation_id: UUID4
    timestamp: datetime
    object_type: ObjectType
    confidence: float
    bounding_box: Optional[BoundingBoxSchema] = None
    location: Optional[LocationSchema] = None
    velocity: Optional[VelocitySchema] = None
    attributes: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class DetectionCreate(DetectionBase):
    """Detection creation schema."""
    pass

class DetectionInDB(DetectionBase):
    """Detection database schema."""
    id: UUID4
    target_id: Optional[UUID4] = None
    created_at: datetime

    class Config:
        from_attributes = True

class Detection(DetectionInDB):
    """Detection schema."""
    pass

# Target schemas
class TargetBase(BaseModel):
    """Base target schema."""
    track_id: str
    object_type: ObjectType
    first_seen: datetime
    last_seen: datetime
    confidence: float
    location: Optional[LocationSchema] = None
    velocity: Optional[VelocitySchema] = None
    attributes: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class TargetCreate(TargetBase):
    """Target creation schema."""
    pass

class TargetUpdate(BaseModel):
    """Target update schema."""
    last_seen: Optional[datetime] = None
    status: Optional[TargetStatus] = None
    confidence: Optional[float] = None
    location: Optional[LocationSchema] = None
    velocity: Optional[VelocitySchema] = None
    attributes: Optional[Dict[str, Any]] = None
    metadata: Optional[Dict[str, Any]] = None

class TargetInDB(TargetBase):
    """Target database schema."""
    id: UUID4
    status: TargetStatus
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class Target(TargetInDB):
    """Target schema."""
    detections: List[Detection] = []

# Surveillance area schemas
class SurveillanceAreaBase(BaseModel):
    """Base surveillance area schema."""
    name: str
    description: Optional[str] = None
    geometry: Dict[str, Any]  # GeoJSON geometry
    priority: int = 0
    active: bool = True
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None

class SurveillanceAreaCreate(SurveillanceAreaBase):
    """Surveillance area creation schema."""
    pass

class SurveillanceAreaUpdate(BaseModel):
    """Surveillance area update schema."""
    name: Optional[str] = None
    description: Optional[str] = None
    geometry: Optional[Dict[str, Any]] = None
    priority: Optional[int] = None
    active: Optional[bool] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None

class SurveillanceAreaInDB(SurveillanceAreaBase):
    """Surveillance area database schema."""
    id: UUID4
    created_by: Optional[str] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class SurveillanceArea(SurveillanceAreaInDB):
    """Surveillance area schema."""
    pass

# Alert schemas
class AlertBase(BaseModel):
    """Base alert schema."""
    type: AlertType
    severity: AlertSeverity
    message: str
    timestamp: datetime
    location: Optional[LocationSchema] = None
    target_id: Optional[UUID4] = None
    surveillance_area_id: Optional[UUID4] = None
    metadata: Optional[Dict[str, Any]] = None

class AlertCreate(AlertBase):
    """Alert creation schema."""
    pass

class AlertUpdate(BaseModel):
    """Alert update schema."""
    acknowledged: bool = True
    acknowledged_by: Optional[str] = None
    acknowledged_at: Optional[datetime] = None

class AlertInDB(AlertBase):
    """Alert database schema."""
    id: UUID4
    acknowledged: bool = False
    acknowledged_by: Optional[str] = None
    acknowledged_at: Optional[datetime] = None
    created_at: datetime

    class Config:
        from_attributes = True

class Alert(AlertInDB):
    """Alert schema."""
    pass
