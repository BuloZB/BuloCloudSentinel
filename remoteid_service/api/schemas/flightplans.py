"""
Flight plan schemas for the Remote ID & Regulatory Compliance Service.

This module contains Pydantic models for flight plan API request and response validation.
"""

from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional, Any, Union
from uuid import UUID

from pydantic import BaseModel, Field, validator

from remoteid_service.api.schemas.remoteid import Position

# Enums
class FlightPlanStatus(str, Enum):
    """Flight plan status."""
    DRAFT = "draft"
    SUBMITTED = "submitted"
    APPROVED = "approved"
    REJECTED = "rejected"
    CANCELLED = "cancelled"
    COMPLETED = "completed"

class FlightPlanType(str, Enum):
    """Flight plan type."""
    EASA_SORA = "easa_sora"
    FAA_LAANC = "faa_laanc"
    CUSTOM = "custom"

class WaypointAction(str, Enum):
    """Waypoint action."""
    TAKEOFF = "takeoff"
    LAND = "land"
    HOVER = "hover"
    PHOTO = "photo"
    VIDEO_START = "video_start"
    VIDEO_STOP = "video_stop"
    CUSTOM = "custom"

# Base models
class Waypoint(BaseModel):
    """Waypoint model."""
    sequence: int
    position: Position
    speed: Optional[float] = None  # m/s
    hold_time: Optional[int] = None  # seconds
    action: Optional[WaypointAction] = None
    parameters: Optional[Dict[str, Any]] = None

class GeoPolygon(BaseModel):
    """Geographic polygon model."""
    points: List[Position]

class GeoPath(BaseModel):
    """Geographic path model."""
    points: List[Position]

# Request models
class CreateFlightPlanRequest(BaseModel):
    """Create flight plan request."""
    name: str
    description: Optional[str] = None
    operator_id: str
    drone_id: str
    plan_type: FlightPlanType
    start_time: datetime
    end_time: datetime
    max_altitude: float  # meters
    area: GeoPolygon
    path: Optional[GeoPath] = None
    waypoints: List[Waypoint]
    metadata: Optional[Dict[str, Any]] = None

class UpdateFlightPlanRequest(BaseModel):
    """Update flight plan request."""
    name: Optional[str] = None
    description: Optional[str] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    max_altitude: Optional[float] = None
    area: Optional[GeoPolygon] = None
    path: Optional[GeoPath] = None
    waypoints: Optional[List[Waypoint]] = None
    metadata: Optional[Dict[str, Any]] = None

class SubmitFlightPlanRequest(BaseModel):
    """Submit flight plan request."""
    flight_plan_id: UUID
    submission_type: FlightPlanType
    additional_data: Optional[Dict[str, Any]] = None

class CancelFlightPlanRequest(BaseModel):
    """Cancel flight plan request."""
    flight_plan_id: UUID
    reason: Optional[str] = None

class GetFlightPlansRequest(BaseModel):
    """Get flight plans request."""
    operator_id: Optional[str] = None
    drone_id: Optional[str] = None
    status: Optional[FlightPlanStatus] = None
    start_time_from: Optional[datetime] = None
    start_time_to: Optional[datetime] = None
    limit: Optional[int] = 100
    offset: Optional[int] = 0

# Response models
class FlightPlanResponse(BaseModel):
    """Flight plan response."""
    id: UUID
    name: str
    description: Optional[str] = None
    operator_id: str
    drone_id: str
    plan_type: FlightPlanType
    status: FlightPlanStatus
    start_time: datetime
    end_time: datetime
    max_altitude: float
    area: GeoPolygon
    path: Optional[GeoPath] = None
    waypoints: List[Waypoint]
    submission_id: Optional[str] = None
    submission_time: Optional[datetime] = None
    approval_time: Optional[datetime] = None
    rejection_reason: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    created_at: datetime
    updated_at: datetime

class FlightPlansResponse(BaseModel):
    """Flight plans response."""
    flight_plans: List[FlightPlanResponse]
    total: int
    limit: int
    offset: int

class FlightPlanSubmissionResponse(BaseModel):
    """Flight plan submission response."""
    flight_plan_id: UUID
    submission_id: str
    submission_time: datetime
    status: FlightPlanStatus
    message: Optional[str] = None
    details: Optional[Dict[str, Any]] = None

# EASA SORA specific models
class EASASoraRiskClass(str, Enum):
    """EASA SORA risk class."""
    OPEN_A1 = "open_a1"
    OPEN_A2 = "open_a2"
    OPEN_A3 = "open_a3"
    SPECIFIC_PDRA = "specific_pdra"
    SPECIFIC_SORA = "specific_sora"
    CERTIFIED = "certified"

class EASASoraOperationalVolume(BaseModel):
    """EASA SORA operational volume."""
    area: GeoPolygon
    min_height: float  # meters
    max_height: float  # meters
    buffer: Optional[float] = None  # meters

class EASASoraSubmission(BaseModel):
    """EASA SORA submission."""
    risk_class: EASASoraRiskClass
    operational_volume: EASASoraOperationalVolume
    start_time: datetime
    end_time: datetime
    drone_details: Dict[str, Any]
    operator_details: Dict[str, Any]
    operation_details: Dict[str, Any]
    mitigations: Optional[Dict[str, Any]] = None

# FAA LAANC specific models
class FAALaancAirspaceClass(str, Enum):
    """FAA LAANC airspace class."""
    B = "b"
    C = "c"
    D = "d"
    E = "e"
    G = "g"

class FAALaancOperationType(str, Enum):
    """FAA LAANC operation type."""
    PART_107 = "part_107"
    RECREATIONAL = "recreational"
    PUBLIC_SAFETY = "public_safety"

class FAALaancSubmission(BaseModel):
    """FAA LAANC submission."""
    operation_type: FAALaancOperationType
    airspace_class: FAALaancAirspaceClass
    area: GeoPolygon
    max_altitude: float  # feet AGL
    start_time: datetime
    end_time: datetime
    pilot_details: Dict[str, Any]
    aircraft_details: Dict[str, Any]
    operation_details: Dict[str, Any]
