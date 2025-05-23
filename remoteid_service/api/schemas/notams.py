"""
NOTAM schemas for the Remote ID & Regulatory Compliance Service.

This module contains Pydantic models for NOTAM API request and response validation.
"""

from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional, Any, Union
from uuid import UUID

from pydantic import BaseModel, Field, validator

from remoteid_service.api.schemas.remoteid import Position
from remoteid_service.api.schemas.flightplans import GeoPolygon

# Enums
class NOTAMType(str, Enum):
    """NOTAM type."""
    AIRSPACE = "airspace"
    OBSTACLE = "obstacle"
    AIRPORT = "airport"
    PROCEDURE = "procedure"
    OTHER = "other"

class NOTAMSource(str, Enum):
    """NOTAM source."""
    FAA = "faa"
    EASA = "easa"
    EUROCONTROL = "eurocontrol"
    ICAO = "icao"
    OTHER = "other"

class NOTAMStatus(str, Enum):
    """NOTAM status."""
    ACTIVE = "active"
    INACTIVE = "inactive"
    UPCOMING = "upcoming"
    EXPIRED = "expired"

# Request models
class ImportNOTAMsRequest(BaseModel):
    """Import NOTAMs request."""
    source: NOTAMSource
    region: Optional[str] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    location_code: Optional[str] = None
    force_update: Optional[bool] = False

class GetNOTAMsRequest(BaseModel):
    """Get NOTAMs request."""
    source: Optional[NOTAMSource] = None
    notam_type: Optional[NOTAMType] = None
    location_code: Optional[str] = None
    status: Optional[NOTAMStatus] = None
    effective_start_from: Optional[datetime] = None
    effective_start_to: Optional[datetime] = None
    effective_end_from: Optional[datetime] = None
    effective_end_to: Optional[datetime] = None
    area: Optional[GeoPolygon] = None
    point: Optional[Position] = None
    radius: Optional[float] = None  # meters
    limit: Optional[int] = 100
    offset: Optional[int] = 0

class CheckFlightPlanNOTAMsRequest(BaseModel):
    """Check flight plan NOTAMs request."""
    flight_plan_id: UUID

# Response models
class NOTAMResponse(BaseModel):
    """NOTAM response."""
    id: UUID
    notam_id: str
    source: NOTAMSource
    notam_type: NOTAMType
    location_code: str
    effective_start: datetime
    effective_end: datetime
    status: NOTAMStatus
    altitude_lower: Optional[float] = None  # meters
    altitude_upper: Optional[float] = None  # meters
    area: Optional[GeoPolygon] = None
    point: Optional[Position] = None
    description: str
    raw_text: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    created_at: datetime
    updated_at: datetime

class NOTAMsResponse(BaseModel):
    """NOTAMs response."""
    notams: List[NOTAMResponse]
    total: int
    limit: int
    offset: int

class NOTAMImportResponse(BaseModel):
    """NOTAM import response."""
    imported_count: int
    updated_count: int
    skipped_count: int
    failed_count: int
    details: Optional[Dict[str, Any]] = None

class NOTAMConflict(BaseModel):
    """NOTAM conflict."""
    notam: NOTAMResponse
    conflict_type: str  # spatial, temporal, altitude
    description: str
    severity: str  # high, medium, low

class FlightPlanNOTAMsResponse(BaseModel):
    """Flight plan NOTAMs response."""
    flight_plan_id: UUID
    conflicts: List[NOTAMConflict]
    has_critical_conflicts: bool

# AIXM 5.1 specific models
class AIXMPoint(BaseModel):
    """AIXM point."""
    latitude: float
    longitude: float
    elevation: Optional[float] = None

class AIXMCircle(BaseModel):
    """AIXM circle."""
    center: AIXMPoint
    radius: float  # meters

class AIXMAirspace(BaseModel):
    """AIXM airspace."""
    designator: str
    name: Optional[str] = None
    type: str
    upper_limit: Optional[float] = None
    lower_limit: Optional[float] = None
    geometry: Union[GeoPolygon, AIXMCircle]

class AIXMTimeSlice(BaseModel):
    """AIXM time slice."""
    valid_from: datetime
    valid_to: Optional[datetime] = None
    interpretation: str  # BASELINE, TEMPDELTA, PERMDELTA

class AIXMFeature(BaseModel):
    """AIXM feature."""
    id: str
    type: str
    time_slice: AIXMTimeSlice
    properties: Dict[str, Any]
