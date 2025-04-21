"""
Geofencing models for Bulo.Cloud Sentinel.

This module provides data models for geofence zones, providers,
and validation responses.
"""

from enum import Enum
from typing import List, Dict, Any, Optional, Union
from pydantic import BaseModel, Field, validator
import uuid
from datetime import datetime
import json

class GeofenceZoneType(str, Enum):
    """Types of geofence zones."""
    AIRPORT = "airport"
    HELIPORT = "heliport"
    CONTROLLED_AIRSPACE = "controlled_airspace"
    SPECIAL_USE_AIRSPACE = "special_use_airspace"
    NATIONAL_PARK = "national_park"
    MILITARY = "military"
    TEMPORARY_FLIGHT_RESTRICTION = "tfr"
    CUSTOM = "custom"
    OTHER = "other"

class GeofenceZoneSource(str, Enum):
    """Sources of geofence data."""
    FAA = "faa"
    EASA = "easa"
    AIRMAP = "airmap"
    OPENSTREETMAP = "openstreetmap"
    CUSTOM = "custom"
    OTHER = "other"

class GeofenceRestrictionLevel(str, Enum):
    """Restriction levels for geofence zones."""
    NO_FLY = "no_fly"  # Absolutely prohibited
    RESTRICTED = "restricted"  # Requires authorization
    CAUTION = "caution"  # Fly with caution
    NOTICE = "notice"  # Notice only, no restriction
    CUSTOM = "custom"  # Custom restriction level

class GeofenceZone(BaseModel):
    """A geofence zone with its boundaries and properties."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: Optional[str] = None
    zone_type: GeofenceZoneType
    source: GeofenceZoneSource
    restriction_level: GeofenceRestrictionLevel
    geometry_type: str  # "polygon", "circle", etc.
    geometry: Dict[str, Any]  # GeoJSON-like structure
    altitude_min: Optional[float] = None  # Meters above sea level
    altitude_max: Optional[float] = None  # Meters above sea level
    effective_from: Optional[datetime] = None
    effective_to: Optional[datetime] = None
    properties: Dict[str, Any] = Field(default_factory=dict)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

class CustomGeofenceCreate(BaseModel):
    """Request model for creating a custom geofence."""
    name: str
    description: Optional[str] = None
    restriction_level: GeofenceRestrictionLevel
    geometry_type: str
    geometry: Dict[str, Any]
    altitude_min: Optional[float] = None
    altitude_max: Optional[float] = None
    effective_from: Optional[datetime] = None
    effective_to: Optional[datetime] = None
    properties: Dict[str, Any] = Field(default_factory=dict)

class CustomGeofenceUpdate(BaseModel):
    """Request model for updating a custom geofence."""
    name: Optional[str] = None
    description: Optional[str] = None
    restriction_level: Optional[GeofenceRestrictionLevel] = None
    geometry_type: Optional[str] = None
    geometry: Optional[Dict[str, Any]] = None
    altitude_min: Optional[float] = None
    altitude_max: Optional[float] = None
    effective_from: Optional[datetime] = None
    effective_to: Optional[datetime] = None
    properties: Optional[Dict[str, Any]] = None

class GeofenceZoneResponse(GeofenceZone):
    """Response model for geofence zones."""
    pass

class GeofenceProvider(BaseModel):
    """A provider of geofence data."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: Optional[str] = None
    url: str
    api_key_required: bool = False
    api_key_configured: bool = False
    last_update: Optional[datetime] = None
    zone_types: List[GeofenceZoneType] = []
    enabled: bool = True

class GeofenceProviderResponse(GeofenceProvider):
    """Response model for geofence providers."""
    pass

class MissionValidationRequest(BaseModel):
    """Request model for validating a mission against geofence zones."""
    mission_id: Optional[str] = None
    waypoints: Optional[List[Dict[str, Any]]] = None

class WaypointValidationRequest(BaseModel):
    """Request model for validating a waypoint against geofence zones."""
    latitude: float
    longitude: float
    altitude: Optional[float] = None

class GeofenceViolation(BaseModel):
    """A violation of a geofence zone."""
    zone_id: str
    zone_name: str
    zone_type: GeofenceZoneType
    restriction_level: GeofenceRestrictionLevel
    waypoint_index: Optional[int] = None
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    description: str

class MissionValidationResponse(BaseModel):
    """Response model for mission validation."""
    valid: bool
    violations: List[GeofenceViolation] = []

class WaypointValidationResponse(BaseModel):
    """Response model for waypoint validation."""
    valid: bool
    violations: List[GeofenceViolation] = []

# Database models
class GeofenceZoneModel(BaseModel):
    """Database model for a geofence zone."""
    id: str
    name: str
    description: Optional[str] = None
    zone_type: str
    source: str
    restriction_level: str
    geometry_type: str
    geometry: str  # JSON string
    altitude_min: Optional[float] = None
    altitude_max: Optional[float] = None
    effective_from: Optional[datetime] = None
    effective_to: Optional[datetime] = None
    properties: str  # JSON string
    created_at: datetime
    updated_at: datetime

class GeofenceProviderModel(BaseModel):
    """Database model for a geofence provider."""
    id: str
    name: str
    description: Optional[str] = None
    url: str
    api_key_required: bool
    api_key_configured: bool
    last_update: Optional[datetime] = None
    zone_types: str  # JSON string
    enabled: bool
