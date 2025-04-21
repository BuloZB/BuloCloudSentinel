"""
Geofencing schemas for the Drone Swarm System.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from enum import Enum

from .drone import Position

class GeofenceType(str, Enum):
    """Geofence type enumeration."""
    INCLUSION = "inclusion"  # Drone must stay inside
    EXCLUSION = "exclusion"  # Drone must stay outside
    ALTITUDE_FLOOR = "altitude_floor"  # Minimum altitude
    ALTITUDE_CEILING = "altitude_ceiling"  # Maximum altitude
    SPEED_LIMIT = "speed_limit"  # Maximum speed in area
    RESTRICTED = "restricted"  # Restricted area (regulatory)
    TEMPORARY = "temporary"  # Temporary restriction

class GeofenceShape(str, Enum):
    """Geofence shape enumeration."""
    CIRCLE = "circle"
    POLYGON = "polygon"
    CYLINDER = "cylinder"

class CircleGeofence(BaseModel):
    """Circle geofence model."""
    center: Position
    radius: float  # meters

class PolygonGeofence(BaseModel):
    """Polygon geofence model."""
    points: List[Position]
    min_altitude: Optional[float] = None
    max_altitude: Optional[float] = None

class CylinderGeofence(BaseModel):
    """Cylinder geofence model."""
    center: Position
    radius: float  # meters
    min_altitude: float
    max_altitude: float

class GeofenceZoneCreate(BaseModel):
    """Model for creating a new geofence zone."""
    name: str
    description: Optional[str] = None
    geofence_type: GeofenceType
    shape: GeofenceShape
    circle: Optional[CircleGeofence] = None
    polygon: Optional[PolygonGeofence] = None
    cylinder: Optional[CylinderGeofence] = None
    parameters: Optional[Dict[str, Any]] = None
    active: bool = True
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    tags: Optional[List[str]] = None

class GeofenceZoneUpdate(BaseModel):
    """Model for updating a geofence zone."""
    name: Optional[str] = None
    description: Optional[str] = None
    geofence_type: Optional[GeofenceType] = None
    shape: Optional[GeofenceShape] = None
    circle: Optional[CircleGeofence] = None
    polygon: Optional[PolygonGeofence] = None
    cylinder: Optional[CylinderGeofence] = None
    parameters: Optional[Dict[str, Any]] = None
    active: Optional[bool] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    tags: Optional[List[str]] = None

class GeofenceZone(BaseModel):
    """Geofence zone model."""
    id: str
    name: str
    description: Optional[str] = None
    geofence_type: GeofenceType
    shape: GeofenceShape
    circle: Optional[CircleGeofence] = None
    polygon: Optional[PolygonGeofence] = None
    cylinder: Optional[CylinderGeofence] = None
    parameters: Optional[Dict[str, Any]] = None
    active: bool = True
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    tags: Optional[List[str]] = None
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

class GeofenceViolation(BaseModel):
    """Geofence violation model."""
    id: str
    drone_id: str
    zone_id: str
    violation_type: str  # entry, exit, altitude, speed
    position: Position
    timestamp: datetime
    details: Optional[Dict[str, Any]] = None

class GeofenceCheckResult(BaseModel):
    """Result of a geofence check."""
    position: Position
    inside_zones: List[str]  # zone IDs
    outside_zones: List[str]  # zone IDs
    violations: List[GeofenceViolation]
    is_allowed: bool
