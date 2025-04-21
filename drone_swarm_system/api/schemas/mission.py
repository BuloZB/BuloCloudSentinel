"""
Mission schemas for the Drone Swarm System.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from enum import Enum

from .drone import Position, Attitude

class MissionType(str, Enum):
    """Mission type enumeration."""
    WAYPOINT = "waypoint"
    SURVEY = "survey"
    CORRIDOR = "corridor"
    PERIMETER = "perimeter"
    SEARCH = "search"
    FOLLOW = "follow"
    CUSTOM = "custom"

class MissionStatus(str, Enum):
    """Mission status enumeration."""
    CREATED = "created"
    PLANNED = "planned"
    VALIDATED = "validated"
    READY = "ready"
    EXECUTING = "executing"
    PAUSED = "paused"
    COMPLETED = "completed"
    ABORTED = "aborted"
    FAILED = "failed"

class WaypointAction(str, Enum):
    """Waypoint action enumeration."""
    NONE = "none"
    TAKE_PHOTO = "take_photo"
    START_VIDEO = "start_video"
    STOP_VIDEO = "stop_video"
    HOVER = "hover"
    LAND = "land"
    TAKEOFF = "takeoff"
    CHANGE_SPEED = "change_speed"
    CHANGE_ALTITUDE = "change_altitude"
    ROTATE = "rotate"
    PAYLOAD_ACTION = "payload_action"
    CUSTOM = "custom"

class Waypoint(BaseModel):
    """Waypoint model."""
    id: Optional[str] = None
    position: Position
    altitude_reference: str = "relative"  # relative, absolute, terrain
    heading: Optional[float] = None  # degrees, None for auto
    speed: Optional[float] = None  # m/s, None for default
    acceptance_radius: Optional[float] = None  # meters
    pass_through: bool = False
    delay: Optional[float] = None  # seconds to wait at waypoint
    action: Optional[WaypointAction] = None
    action_params: Optional[Dict[str, Any]] = None

class SurveyArea(BaseModel):
    """Survey area model."""
    boundary: List[Position]
    altitude: float
    altitude_reference: str = "relative"  # relative, absolute, terrain
    speed: Optional[float] = None  # m/s
    overlap_horizontal: float = 70.0  # percent
    overlap_vertical: float = 70.0  # percent
    angle: float = 0.0  # degrees
    camera_trigger_distance: Optional[float] = None  # meters
    camera_params: Optional[Dict[str, Any]] = None

class CorridorSurvey(BaseModel):
    """Corridor survey model."""
    centerline: List[Position]
    width: float  # meters
    altitude: float
    altitude_reference: str = "relative"
    speed: Optional[float] = None  # m/s
    overlap_horizontal: float = 70.0  # percent
    overlap_vertical: float = 70.0  # percent
    camera_trigger_distance: Optional[float] = None  # meters
    camera_params: Optional[Dict[str, Any]] = None

class SearchPattern(BaseModel):
    """Search pattern model."""
    area: List[Position]
    altitude: float
    altitude_reference: str = "relative"
    speed: Optional[float] = None  # m/s
    pattern: str = "parallel"  # parallel, spiral, expanding_square
    spacing: float  # meters
    direction: float = 0.0  # degrees

class MissionConstraints(BaseModel):
    """Mission constraints model."""
    max_altitude: Optional[float] = None  # meters
    min_altitude: Optional[float] = None  # meters
    max_distance: Optional[float] = None  # meters from home
    max_speed: Optional[float] = None  # m/s
    max_flight_time: Optional[float] = None  # minutes
    require_gps_lock: bool = True
    require_min_battery: Optional[float] = None  # percent
    avoid_no_fly_zones: bool = True
    check_weather: bool = True
    max_wind_speed: Optional[float] = None  # m/s
    min_visibility: Optional[float] = None  # meters
    avoid_precipitation: bool = True

class MissionState(BaseModel):
    """Mission state model."""
    mission_id: str
    status: MissionStatus
    progress: float = 0.0  # 0-100%
    current_waypoint: Optional[int] = None
    drone_ids: List[str] = []
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    duration: Optional[float] = None  # seconds
    distance_traveled: Optional[float] = None  # meters
    battery_used: Optional[float] = None  # percent
    errors: List[str] = []
    warnings: List[str] = []
    last_updated: datetime = Field(default_factory=datetime.now)

class MissionCreate(BaseModel):
    """Model for creating a new mission."""
    name: str
    mission_type: MissionType
    description: Optional[str] = None
    waypoints: Optional[List[Waypoint]] = None
    survey_area: Optional[SurveyArea] = None
    corridor_survey: Optional[CorridorSurvey] = None
    search_pattern: Optional[SearchPattern] = None
    constraints: Optional[MissionConstraints] = None
    parameters: Optional[Dict[str, Any]] = None
    tags: Optional[List[str]] = None

class MissionUpdate(BaseModel):
    """Model for updating a mission."""
    name: Optional[str] = None
    mission_type: Optional[MissionType] = None
    description: Optional[str] = None
    waypoints: Optional[List[Waypoint]] = None
    survey_area: Optional[SurveyArea] = None
    corridor_survey: Optional[CorridorSurvey] = None
    search_pattern: Optional[SearchPattern] = None
    constraints: Optional[MissionConstraints] = None
    parameters: Optional[Dict[str, Any]] = None
    tags: Optional[List[str]] = None

class Mission(BaseModel):
    """Mission model."""
    id: str
    name: str
    mission_type: MissionType
    description: Optional[str] = None
    waypoints: Optional[List[Waypoint]] = None
    survey_area: Optional[SurveyArea] = None
    corridor_survey: Optional[CorridorSurvey] = None
    search_pattern: Optional[SearchPattern] = None
    constraints: Optional[MissionConstraints] = None
    parameters: Optional[Dict[str, Any]] = None
    tags: Optional[List[str]] = None
    status: MissionStatus = MissionStatus.CREATED
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
